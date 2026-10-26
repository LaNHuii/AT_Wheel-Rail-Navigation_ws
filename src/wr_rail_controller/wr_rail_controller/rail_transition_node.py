#!/usr/bin/env python3
"""进轨/离轨 Action 服务节点。

节点汇集轨道入口估计、夹轨反馈、底盘状态和雷达安全状态，
执行严格顺序的局部过渡状态机。过渡期间速度只发布到 /cmd_vel_transition。
进轨速度、容差和行程位于 config/rail_controller.yaml；夹轨电流/转矩保护由 STM32 完成。
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
from wr_interfaces.action import ExecuteRailTransition
from wr_interfaces.msg import (
    ChassisControlCommand,
    ChassisState,
    LocalSafetyState,
    RailClampCommand,
    RailClampFeedback,
    RailEntryEstimate,
)

from .runtime_transition import (
    ALIGN,
    APPROACH,
    CLAMP_NONE,
    COMPLETE,
    CONFIRMED,
    EXIT_FORWARD,
    EXIT_RELEASE,
    FAILED,
    FINAL_PROBE,
    FRONT_PROBE,
    FRONT_RELEASE,
    REAR_ADVANCE,
    RuntimeRailTransition,
    RuntimeTransitionConfig,
    RuntimeTransitionInput,
)


PHASE_CODES = {
    ALIGN: ExecuteRailTransition.Feedback.PHASE_ALIGN,
    APPROACH: ExecuteRailTransition.Feedback.PHASE_APPROACH,
    FRONT_PROBE: ExecuteRailTransition.Feedback.PHASE_FRONT_PROBE,
    FRONT_RELEASE: ExecuteRailTransition.Feedback.PHASE_FRONT_RELEASE,
    REAR_ADVANCE: ExecuteRailTransition.Feedback.PHASE_REAR_ADVANCE,
    FINAL_PROBE: ExecuteRailTransition.Feedback.PHASE_FINAL_PROBE,
    CONFIRMED: ExecuteRailTransition.Feedback.PHASE_CONFIRMED,
    EXIT_RELEASE: ExecuteRailTransition.Feedback.PHASE_EXITING,
    EXIT_FORWARD: ExecuteRailTransition.Feedback.PHASE_EXITING,
    COMPLETE: ExecuteRailTransition.Feedback.PHASE_CONFIRMED,
    FAILED: ExecuteRailTransition.Feedback.PHASE_FAILED,
}


class RailTransitionNode(Node):
    def __init__(self):
        super().__init__('rail_transition_controller')
        # 过渡参数由 YAML 覆盖。estimate/feedback/chassis_timeout 越小停车越及时，
        # 但网络抖动时更容易误停；调参位置为 config/rail_controller.yaml。
        defaults = RuntimeTransitionConfig()
        for name in defaults.__dataclass_fields__:
            self.declare_parameter(name, getattr(defaults, name))
        self.declare_parameter('estimate_timeout', 0.30)
        self.declare_parameter('feedback_timeout', 0.25)
        self.declare_parameter('chassis_timeout', 0.25)
        self.declare_parameter('control_rate', 20.0)
        values = {}
        for name, field in defaults.__dataclass_fields__.items():
            value = self.get_parameter(name).value
            values[name] = int(value) if field.type is int else float(value)
        self.config = RuntimeTransitionConfig(**values)
        self.estimate_timeout = float(self.get_parameter('estimate_timeout').value)
        self.feedback_timeout = float(self.get_parameter('feedback_timeout').value)
        self.chassis_timeout = float(self.get_parameter('chassis_timeout').value)
        self.control_rate = float(self.get_parameter('control_rate').value)
        self.lock = threading.RLock()
        self.active_goal = None
        self.machine = None
        self.started_at = 0.0
        self.phase_started_at = 0.0
        self.travelled = 0.0
        self.last_integrated_at = time.monotonic()
        self.estimate = None
        self.feedback = None
        self.chassis = None
        self.safety = None
        self.estimate_at = self.feedback_at = self.chassis_at = self.safety_at = float('-inf')
        self.last_output = None
        callback_group = ReentrantCallbackGroup()
        self.velocity_publisher = self.create_publisher(Twist, 'cmd_vel_transition', 10)
        self.clamp_publisher = self.create_publisher(RailClampCommand, 'rail_clamp_request', 10)
        self.control_publisher = self.create_publisher(
            ChassisControlCommand, 'chassis_control_command', 10)
        self.create_subscription(
            RailEntryEstimate, 'rail_entry_estimate', self.on_estimate, 10,
            callback_group=callback_group)
        self.create_subscription(
            RailClampFeedback, 'rail_clamp_feedback', self.on_feedback, 10,
            callback_group=callback_group)
        self.create_subscription(
            ChassisState, 'chassis_state', self.on_chassis, 10,
            callback_group=callback_group)
        self.create_subscription(
            LocalSafetyState, 'local_safety_state', self.on_safety, 10,
            callback_group=callback_group)
        self.action_server = ActionServer(
            self, ExecuteRailTransition, 'execute_rail_transition', self.execute,
            goal_callback=self.on_goal, cancel_callback=self.on_cancel,
            callback_group=callback_group)
        self.timer = self.create_timer(
            1.0 / self.control_rate, self.on_timer, callback_group=callback_group)

    def on_estimate(self, message):
        with self.lock:
            self.estimate = message
            self.estimate_at = time.monotonic()

    def on_feedback(self, message):
        with self.lock:
            self.feedback = message
            self.feedback_at = time.monotonic()

    def on_chassis(self, message):
        with self.lock:
            now = time.monotonic()
            if self.active_goal is not None and self.chassis is not None:
                dt = max(0.0, now - self.last_integrated_at)
                self.travelled += abs(float(message.measured_twist.linear.x)) * dt
            self.last_integrated_at = now
            self.chassis = message
            self.chassis_at = now

    def on_safety(self, message):
        with self.lock:
            self.safety = message
            self.safety_at = time.monotonic()

    def on_goal(self, request):
        with self.lock:
            if self.active_goal is not None:
                return GoalResponse.REJECT
        if request.operation not in (
                ExecuteRailTransition.Goal.ENTER,
                ExecuteRailTransition.Goal.EXIT_FORWARD):
            return GoalResponse.REJECT
        if not math.isfinite(request.timeout) or request.timeout <= 0.0:
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    @staticmethod
    def on_cancel(_goal_handle):
        return CancelResponse.ACCEPT

    def publish_zero(self):
        self.velocity_publisher.publish(Twist())

    def publish_clamp(self, action, mask):
        # 夹轨上位机只给出最大探测角、速度和超时；
        # 单轮接触后停止加角、硬过流与堵转保护必须由 STM32 实时执行。
        # 这些数值待实车标定后应迁入 rail_controller.yaml。
        message = RailClampCommand()
        message.header.stamp = self.get_clock().now().to_msg()
        message.sequence_id = int(time.monotonic_ns() & 0xffffffff)
        message.action = action
        message.module_mask = mask
        message.max_probe_angle = math.radians(10.0)
        message.probe_angular_speed = math.radians(5.0)
        message.timeout = 5.0
        self.clamp_publisher.publish(message)

    def request_mode(self, mode):
        message = ChassisControlCommand()
        message.header.stamp = self.get_clock().now().to_msg()
        message.sequence_id = int(time.monotonic_ns() & 0xffffffff)
        message.enable = True
        message.requested_control_source = ChassisControlCommand.SOURCE_AUTONOMOUS
        message.requested_mode = mode
        message.valid_for_ms = 200
        self.control_publisher.publish(message)

    def execute(self, goal_handle):
        # Action 超时或取消时先发零速度，再结束目标，防止保留旧指令。
        request = goal_handle.request
        operation = 'enter' if request.operation == request.ENTER else 'exit'
        with self.lock:
            self.active_goal = goal_handle
            self.machine = RuntimeRailTransition(operation, self.config)
            self.started_at = time.monotonic()
            self.travelled = 0.0
            self.last_integrated_at = self.started_at
            self.last_output = None
            self.request_mode(
                ChassisControlCommand.MODE_RAIL_ENTRY
                if operation == 'enter' else ChassisControlCommand.MODE_RAIL_EXIT)
        result = ExecuteRailTransition.Result()
        while rclpy.ok():
            with self.lock:
                output = self.last_output
                elapsed = time.monotonic() - self.started_at
            if goal_handle.is_cancel_requested:
                self.publish_zero()
                self.publish_clamp(RailClampCommand.CANCEL, RailClampCommand.ALL_MODULES)
                with self.lock:
                    self.active_goal = None
                    self.machine = None
                goal_handle.canceled()
                result.result_code = result.RESULT_CANCELLED
                result.message = 'transition cancelled and stopped'
                return result
            if elapsed > float(request.timeout):
                self.publish_zero()
                with self.lock:
                    self.active_goal = None
                    self.machine = None
                goal_handle.abort()
                result.result_code = result.RESULT_TIMEOUT
                result.message = 'transition timed out'
                return result
            if output is not None:
                feedback = ExecuteRailTransition.Feedback()
                feedback.phase = PHASE_CODES.get(output.phase, feedback.PHASE_IDLE)
                feedback.active_module_mask = int(output.module_mask)
                feedback.message = output.reason or output.phase
                goal_handle.publish_feedback(feedback)
                if output.failed or output.done:
                    self.publish_zero()
                    if output.done:
                        final_mode = (
                            ChassisState.MODE_ON_RAIL if operation == 'enter'
                            else ChassisState.MODE_FREE_SURFACE)
                        self.request_mode(final_mode)
                    else:
                        final_mode = (
                            ChassisState.MODE_RAIL_ENTRY if operation == 'enter'
                            else ChassisState.MODE_RAIL_EXIT)
                    with self.lock:
                        self.active_goal = None
                        self.machine = None
                    result.success = bool(output.done)
                    result.result_code = (
                        result.RESULT_SUCCESS if output.done else result.RESULT_CLAMP_FAILED)
                    result.final_mode = final_mode
                    result.message = output.reason or output.phase
                    if output.done:
                        goal_handle.succeed()
                    else:
                        goal_handle.abort()
                    return result
            time.sleep(0.05)
        self.publish_zero()
        goal_handle.abort()
        result.result_code = result.RESULT_HARDWARE_FAULT
        result.message = 'ROS shutdown'
        return result

    def current_input(self, now):
        # 任一必要反馈过期都按不安全处理，不对缺失数据进行猜测。
        estimate_fresh = self.estimate is not None and now - self.estimate_at <= self.estimate_timeout
        feedback_fresh = self.feedback is not None and now - self.feedback_at <= self.feedback_timeout
        chassis_fresh = self.chassis is not None and now - self.chassis_at <= self.chassis_timeout
        safety_fresh = self.safety is not None and now - self.safety_at <= self.feedback_timeout
        data = RuntimeTransitionInput(travelled=self.travelled)
        if estimate_fresh:
            data.estimate_valid = self.estimate.confidence > 0.0
            data.entry_x = float(self.estimate.entry_pose.pose.position.x)
            data.lateral_error = float(self.estimate.lateral_error)
            data.heading_error = float(self.estimate.heading_error)
            data.traversable = self.estimate.traversability == RailEntryEstimate.PASSABLE
        if feedback_fresh:
            data.clamp_state = int(self.feedback.state)
            data.clamp_mask = int(self.feedback.active_module_mask)
            data.contacts = tuple(bool(value) for value in self.feedback.contact_confirmed)
        if safety_fresh:
            data.safety_blocked = bool(
                not self.safety.data_fresh or self.safety.emergency_stop
                or self.safety.forward_path_blocked)
        if not chassis_fresh:
            data.safety_blocked = True
        return data

    def on_timer(self):
        with self.lock:
            if self.active_goal is None or self.machine is None:
                return
            output = self.machine.update(self.current_input(time.monotonic()))
            self.last_output = output
            command = Twist()
            command.linear.x = float(output.vx)
            command.linear.y = float(output.vy)
            command.angular.z = float(output.wz)
            self.velocity_publisher.publish(command)
            if output.clamp_action != CLAMP_NONE:
                self.publish_clamp(output.clamp_action, output.module_mask)


def main(args=None):
    rclpy.init(args=args)
    node = RailTransitionNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.publish_zero()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
