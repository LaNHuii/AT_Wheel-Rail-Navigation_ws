#!/usr/bin/env python3
"""人物跟随端到端仿真验收程序。

程序使用模拟的人员观测、安全状态和轨道入口估计，验证控制编排、
进轨顺序、轨内跟随、离轨和遥控优先级。
通过本测试只表示仿真接口和逻辑闭环通过，不等同于真实视觉、雷达或电控已验证。
"""

import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, UInt8
from wr_interfaces.action import ExecuteRailTransition
from wr_interfaces.msg import (
    ChassisControlCommand,
    ChassisState,
    LocalSafetyState,
    RailEntryEstimate,
    TrackedPerson,
)


class WorkflowVerifier(Node):
    def __init__(self):
        super().__init__('verify_person_follow_workflow')
        self.target = TrackedPerson()
        self.safety = LocalSafetyState()
        self.estimate = RailEntryEstimate()
        self.chassis = None
        self.command = Twist()
        self.target_publisher = self.create_publisher(TrackedPerson, 'tracked_person', 10)
        self.safety_publisher = self.create_publisher(
            LocalSafetyState, 'local_safety_state', 10)
        self.estimate_publisher = self.create_publisher(
            RailEntryEstimate, 'rail_entry_estimate', 10)
        self.transition_publisher = self.create_publisher(
            UInt8, 'rail_transition_request', 10)
        self.manual_publisher = self.create_publisher(Bool, 'sim/manual_override', 10)
        self.autonomy_publisher = self.create_publisher(Bool, 'autonomy_enable', 10)
        self.control_publisher = self.create_publisher(
            ChassisControlCommand, 'chassis_control_command', 10)
        self.create_subscription(ChassisState, 'chassis_state', self.on_chassis, 20)
        self.create_subscription(Twist, 'cmd_vel_chassis', self.on_command, 20)
        self.timer = self.create_timer(0.05, self.publish_inputs)
        self.configure_clear_follow()

    def on_chassis(self, message):
        self.chassis = message

    def on_command(self, message):
        self.command = message

    def publish_inputs(self):
        stamp = self.get_clock().now().to_msg()
        self.target.header.stamp = stamp
        self.target.header.frame_id = 'base_link'
        self.safety.header.stamp = stamp
        self.safety.header.frame_id = 'base_link'
        self.estimate.header.stamp = stamp
        self.estimate.header.frame_id = 'base_link'
        self.target_publisher.publish(self.target)
        self.safety_publisher.publish(self.safety)
        self.estimate_publisher.publish(self.estimate)

    def configure_clear_follow(self, gap=0.80, longitudinal=None):
        self.target.session_id = 1
        self.target.binding_id = 1
        self.target.tracker_id = 1
        self.target.tracking_state = TrackedPerson.TRACKED
        self.target.activity = TrackedPerson.MOVING
        self.target.position.x = float(max(gap, 0.1))
        self.target.position.y = 0.0
        self.target.nearest_body_distance = float(gap)
        self.target.longitudinal_distance = float(
            gap if longitudinal is None else longitudinal)
        self.target.lateral_offset = 0.0
        self.target.detection_confidence = 1.0
        self.target.identity_confidence = 1.0
        self.target.depth_valid_fraction = 1.0
        self.target.original_target = True
        self.safety.data_fresh = True
        self.safety.emergency_stop = False
        self.safety.forward_path_blocked = False
        self.safety.bypass_active = False
        self.safety.rotation_safe = True
        self.safety.nearest_obstacle_distance = 2.0
        self.safety.suggested_lateral_velocity = 0.0
        self.estimate.traversability = RailEntryEstimate.UNKNOWN
        self.estimate.confidence = 0.0

    def wait_for(self, predicate, timeout, label):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if predicate():
                self.get_logger().info('PASS: ' + label)
                return
        raise RuntimeError('timeout: ' + label)

    def run(self):
        self.wait_for(lambda: self.chassis is not None, 8.0, 'simulated chassis state available')
        self.wait_for(lambda: self.command.linear.x > 0.02, 8.0, 'free-surface forward follow')

        self.configure_clear_follow(gap=0.10)
        self.wait_for(
            lambda: abs(self.command.linear.x) < 0.005,
            3.0, 'free surface never reverses toward person')

        self.configure_clear_follow(gap=0.80)
        self.safety.forward_path_blocked = True
        self.safety.bypass_active = True
        self.safety.suggested_lateral_velocity = 0.06
        self.wait_for(
            lambda: self.command.linear.y > 0.04 and self.command.linear.x > 0.0,
            3.0, 'bounded free-surface lateral bypass')

        self.configure_clear_follow(gap=1.0)
        self.target.position.x = 1.0
        self.estimate.entry_pose.pose.position.x = 0.10
        self.estimate.rail_direction.x = 1.0
        self.estimate.rail_spacing = 0.60
        self.estimate.visible_length = 3.0
        self.estimate.lateral_error = 0.0
        self.estimate.heading_error = 0.0
        self.estimate.confidence = 0.95
        self.estimate.traversability = RailEntryEstimate.PASSABLE
        self.wait_for(
            lambda: self.chassis.active_mode == ChassisState.MODE_RAIL_ENTRY,
            8.0, 'behavior tree transfers control to rail entry')
        self.wait_for(
            lambda: self.chassis.active_mode == ChassisState.MODE_ON_RAIL,
            30.0, 'front-pair then four-module clamp entry completes')

        self.configure_clear_follow(gap=0.80, longitudinal=0.80)
        self.wait_for(
            lambda: self.command.linear.x > 0.02 and
            abs(self.command.linear.y) < 0.001 and abs(self.command.angular.z) < 0.001,
            6.0, 'rail forward longitudinal follow')

        self.configure_clear_follow(gap=0.10, longitudinal=0.10)
        self.wait_for(
            lambda: self.command.linear.x < -0.01 and
            abs(self.command.linear.y) < 0.001 and abs(self.command.angular.z) < 0.001,
            4.0, 'rail low-speed reverse gap keeping')

        self.configure_clear_follow(gap=0.80, longitudinal=0.80)
        self.safety.forward_path_blocked = True
        self.wait_for(
            lambda: abs(self.command.linear.x) < 0.005 and
            abs(self.command.linear.y) < 0.005,
            3.0, 'rail obstacle causes hold without bypass')

        self.configure_clear_follow(gap=0.80, longitudinal=0.80)
        request = UInt8()
        request.data = ExecuteRailTransition.Goal.EXIT_FORWARD
        for _ in range(5):
            self.transition_publisher.publish(request)
            rclpy.spin_once(self, timeout_sec=0.05)
        self.wait_for(
            lambda: self.chassis.active_mode == ChassisState.MODE_RAIL_EXIT,
            6.0, 'behavior tree transfers control to forward rail exit')
        self.wait_for(
            lambda: self.chassis.active_mode == ChassisState.MODE_FREE_SURFACE,
            30.0, 'forward rail exit completes')

        manual = Bool(data=True)
        self.manual_publisher.publish(manual)
        self.wait_for(
            lambda: self.chassis.active_control_source == ChassisState.SOURCE_MANUAL and
            abs(self.command.linear.x) < 0.005,
            4.0, 'remote takeover clears autonomous command')
        self.manual_publisher.publish(Bool(data=False))
        self.wait_for(
            lambda: self.chassis.active_control_source == ChassisState.SOURCE_NONE,
            3.0, 'remote release does not resume old command')

        grant = ChassisControlCommand()
        grant.header.stamp = self.get_clock().now().to_msg()
        grant.sequence_id = 100
        grant.enable = True
        grant.requested_control_source = ChassisControlCommand.SOURCE_AUTONOMOUS
        grant.requested_mode = ChassisControlCommand.MODE_FREE_SURFACE
        grant.valid_for_ms = 200
        self.control_publisher.publish(grant)
        self.wait_for(
            lambda: self.chassis.active_control_source == ChassisState.SOURCE_AUTONOMOUS,
            3.0, 'STM32-side autonomous grant restored')
        for _ in range(5):
            self.autonomy_publisher.publish(Bool(data=True))
            rclpy.spin_once(self, timeout_sec=0.05)
        self.wait_for(
            lambda: self.command.linear.x > 0.02,
            8.0, 'explicit host rearm restarts follow')


def main(args=None):
    rclpy.init(args=args)
    node = WorkflowVerifier()
    try:
        node.run()
    except Exception as error:
        node.get_logger().error('FAIL: %s' % error)
        node.destroy_node()
        rclpy.shutdown()
        return 1
    node.get_logger().info('ALL PERSON-FOLLOW WORKFLOW CHECKS PASSED')
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
