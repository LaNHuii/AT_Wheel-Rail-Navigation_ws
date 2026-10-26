#!/usr/bin/env python3
"""人物跟随 ROS 2 Action 节点。

节点将视觉人员观测、MID-360S 安全状态和底盘状态交给局部状态机，
并在 /cmd_vel_follow 输出候选速度。最终控制权由速度仲裁器和 STM32 决定。
调车参数位于 config/person_following.yaml。
"""

import math
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from wr_interfaces.action import FollowPerson
from wr_interfaces.msg import ChassisState, LocalSafetyState, PersonFollowStatus, TrackedPerson

from .follow_logic import FollowConfig, FollowInputs, PersonFollowLogic


class PersonFollowNode(Node):
    """由有界局部跟随状态机支持的行为树 Action 端点。"""

    def __init__(self):
        super().__init__('person_follow_controller')
        # 跟随距离、速度上限、控制增益和数据超时均由 YAML 覆盖。
        # 调参时优先修改 config/person_following.yaml，不要在此处更改默认值。
        defaults = FollowConfig()
        self.declare_parameter('desired_front_gap', defaults.desired_gap)
        self.declare_parameter('gap_deadband', defaults.gap_deadband)
        self.declare_parameter('lost_wait_timeout', defaults.lost_wait_timeout)
        self.declare_parameter('search_radius', defaults.search_radius)
        self.declare_parameter('search_angular_speed', defaults.search_angular_speed)
        self.declare_parameter('max_free_vx', defaults.max_free_vx)
        self.declare_parameter('max_free_vy', defaults.max_free_vy)
        self.declare_parameter('max_rail_vx', defaults.max_rail_vx)
        self.declare_parameter('max_rail_reverse_vx', defaults.max_rail_reverse_vx)
        self.declare_parameter('max_wz', defaults.max_wz)
        self.declare_parameter('distance_gain', defaults.distance_gain)
        self.declare_parameter('lateral_gain', defaults.lateral_gain)
        self.declare_parameter('heading_gain', defaults.heading_gain)
        self.declare_parameter(
            'max_heading_for_translation', defaults.max_heading_for_translation)
        self.declare_parameter('target_timeout', 0.30)
        self.declare_parameter('safety_timeout', 0.25)
        self.declare_parameter('chassis_timeout', 0.25)
        self.declare_parameter('control_rate', 20.0)

        self.base_config = FollowConfig(
            desired_gap=float(self.get_parameter('desired_front_gap').value),
            gap_deadband=float(self.get_parameter('gap_deadband').value),
            lost_wait_timeout=float(self.get_parameter('lost_wait_timeout').value),
            search_radius=float(self.get_parameter('search_radius').value),
            search_angular_speed=float(self.get_parameter('search_angular_speed').value),
            max_free_vx=float(self.get_parameter('max_free_vx').value),
            max_free_vy=float(self.get_parameter('max_free_vy').value),
            max_rail_vx=float(self.get_parameter('max_rail_vx').value),
            max_rail_reverse_vx=float(
                self.get_parameter('max_rail_reverse_vx').value),
            max_wz=float(self.get_parameter('max_wz').value),
            distance_gain=float(self.get_parameter('distance_gain').value),
            lateral_gain=float(self.get_parameter('lateral_gain').value),
            heading_gain=float(self.get_parameter('heading_gain').value),
            max_heading_for_translation=float(
                self.get_parameter('max_heading_for_translation').value),
        )
        self.target_timeout = float(self.get_parameter('target_timeout').value)
        self.safety_timeout = float(self.get_parameter('safety_timeout').value)
        self.chassis_timeout = float(self.get_parameter('chassis_timeout').value)
        control_rate = float(self.get_parameter('control_rate').value)
        positive = (
            self.base_config.desired_gap, self.base_config.gap_deadband,
            self.base_config.lost_wait_timeout, self.base_config.search_radius,
            self.base_config.search_angular_speed, self.base_config.max_free_vx,
            self.base_config.max_free_vy, self.base_config.max_rail_vx,
            self.base_config.max_rail_reverse_vx, self.base_config.max_wz,
            self.target_timeout, self.safety_timeout, self.chassis_timeout, control_rate)
        if not all(math.isfinite(value) and value > 0.0 for value in positive):
            raise ValueError('person-following distances, speeds, rates, and timeouts must be positive')

        self.logic = PersonFollowLogic(self.base_config)
        self.lock = threading.RLock()
        self.target = None
        self.safety = None
        self.chassis = None
        self.target_received_at = float('-inf')
        self.safety_received_at = float('-inf')
        self.chassis_received_at = float('-inf')
        self.active_goal = None
        self.last_output = self.logic.update(FollowInputs(now=time.monotonic()))

        callback_group = ReentrantCallbackGroup()
        self.cmd_publisher = self.create_publisher(Twist, 'cmd_vel_follow', 10)
        self.status_publisher = self.create_publisher(PersonFollowStatus, 'person_follow_status', 10)
        self.create_subscription(
            TrackedPerson, 'tracked_person', self.on_target, 10,
            callback_group=callback_group)
        self.create_subscription(
            LocalSafetyState, 'local_safety_state', self.on_safety, 10,
            callback_group=callback_group)
        self.create_subscription(
            ChassisState, 'chassis_state', self.on_chassis, 10,
            callback_group=callback_group)
        self.action_server = ActionServer(
            self, FollowPerson, 'follow_person', self.execute,
            goal_callback=self.on_goal, cancel_callback=self.on_cancel,
            callback_group=callback_group)
        self.timer = self.create_timer(
            1.0 / control_rate, self.on_timer, callback_group=callback_group)
        self.get_logger().info(
            'Person-follow action ready; commands are isolated on /cmd_vel_follow')

    def on_target(self, message):
        with self.lock:
            self.target = message
            self.target_received_at = time.monotonic()

    def on_safety(self, message):
        with self.lock:
            self.safety = message
            self.safety_received_at = time.monotonic()

    def on_chassis(self, message):
        with self.lock:
            self.chassis = message
            self.chassis_received_at = time.monotonic()

    def on_goal(self, goal_request):
        with self.lock:
            if self.active_goal is not None:
                return GoalResponse.REJECT
        values = (
            goal_request.desired_front_gap,
            goal_request.lost_wait_timeout,
            goal_request.search_radius,
        )
        if not all(math.isfinite(value) and value > 0.0 for value in values):
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    @staticmethod
    def on_cancel(_goal_handle):
        return CancelResponse.ACCEPT

    def execute(self, goal_handle):
        request = goal_handle.request
        config = FollowConfig(**vars(self.base_config))
        config.requested_session_id = int(request.session_id)
        config.desired_gap = float(request.desired_front_gap)
        config.lost_wait_timeout = float(request.lost_wait_timeout)
        config.search_radius = float(request.search_radius)
        config.allow_target_rebind = bool(request.allow_target_rebind)
        with self.lock:
            self.active_goal = goal_handle
            self.logic.start(config)
        result = FollowPerson.Result()
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                with self.lock:
                    final_binding = self.logic.bound_binding_id
                    self.logic.stop()
                    self.active_goal = None
                    self.publish_zero()
                goal_handle.canceled()
                result.success = True
                result.result_code = FollowPerson.Result.RESULT_CANCELLED
                result.final_binding_id = final_binding
                result.message = 'follow action cancelled and chassis command cleared'
                return result
            with self.lock:
                output = self.last_output
                binding_id = self.logic.bound_binding_id
            feedback = FollowPerson.Feedback()
            feedback.state = output.state
            feedback.binding_id = binding_id
            feedback.measured_gap = float(output.measured_gap)
            feedback.message = output.reason
            goal_handle.publish_feedback(feedback)
            time.sleep(0.10)
        with self.lock:
            final_binding = self.logic.bound_binding_id
            self.logic.stop()
            self.active_goal = None
            self.publish_zero()
        goal_handle.abort()
        result.success = False
        result.result_code = FollowPerson.Result.RESULT_STOPPED
        result.final_binding_id = final_binding
        result.message = 'ROS shutdown'
        return result

    def current_inputs(self, now):
        # 三类输入分别超时。target_timeout、safety_timeout 和 chassis_timeout
        # 过小会导致频繁停车，过大会延迟故障停车，均在跟随 YAML 中调整。
        target_fresh = self.target is not None and now - self.target_received_at <= self.target_timeout
        safety_fresh = (
            self.safety is not None and now - self.safety_received_at <= self.safety_timeout
            and self.safety.data_fresh)
        chassis_fresh = self.chassis is not None and (
            now - self.chassis_received_at <= self.chassis_timeout)
        chassis_ok = chassis_fresh and self.chassis.enabled and not (
            self.chassis.physical_estop or self.chassis.command_timed_out
            or self.chassis.chassis_fault_bits)
        data = FollowInputs(
            now=now,
            chassis_mode=(self.chassis.active_mode if chassis_fresh else ChassisState.MODE_FREE_SURFACE),
            chassis_ok=bool(chassis_ok),
            safety_fresh=bool(safety_fresh),
        )
        if not target_fresh:
            # 相机无数据或数据过期属于传感器故障，不是人员丢失，
            # 因此不得由此启动人员丢失等待计时。
            data.target_state = TrackedPerson.SENSOR_FAULT
        if safety_fresh:
            data.emergency_stop = bool(self.safety.emergency_stop)
            data.forward_path_blocked = bool(self.safety.forward_path_blocked)
            data.rotation_safe = bool(self.safety.rotation_safe)
            data.bypass_active = bool(self.safety.bypass_active)
            data.suggested_lateral_velocity = float(
                self.safety.suggested_lateral_velocity)
        if target_fresh:
            data.target_state = int(self.target.tracking_state)
            data.target_session_id = int(self.target.session_id)
            data.binding_id = int(self.target.binding_id)
            data.tracker_id = int(self.target.tracker_id)
            data.activity = int(self.target.activity)
            data.position_x = float(self.target.position.x)
            data.lateral_offset = float(self.target.lateral_offset)
            data.nearest_body_distance = float(self.target.nearest_body_distance)
            data.longitudinal_distance = float(self.target.longitudinal_distance)
            data.original_target = bool(self.target.original_target)
        return data

    def on_timer(self):
        with self.lock:
            if self.active_goal is None:
                return
            now = time.monotonic()
            inputs = self.current_inputs(now)
            output = self.logic.update(inputs)
            self.last_output = output
            command = Twist()
            command.linear.x = float(output.vx)
            command.linear.y = float(output.vy)
            command.angular.z = float(output.wz)
            self.cmd_publisher.publish(command)
            status = PersonFollowStatus()
            status.header.stamp = self.get_clock().now().to_msg()
            status.header.frame_id = 'base_link'
            status.state = output.state
            status.session_id = self.logic.config.requested_session_id
            status.binding_id = self.logic.bound_binding_id
            status.tracker_id = inputs.tracker_id
            status.chassis_mode = inputs.chassis_mode
            status.desired_gap = float(self.logic.config.desired_gap)
            status.measured_gap = float(output.measured_gap)
            status.target_valid = output.target_valid
            status.command_active = any(
                abs(value) > 1.0e-6 for value in (output.vx, output.vy, output.wz))
            status.reason = output.reason
            self.status_publisher.publish(status)

    def publish_zero(self):
        self.cmd_publisher.publish(Twist())

    def destroy_node(self):
        self.action_server.destroy()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PersonFollowNode()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.publish_zero()
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
