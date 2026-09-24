#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from wr_interfaces.msg import (
    RailClampCommand,
    RailClampFeedback,
    RailEngagementState,
)
from wr_interfaces.srv import SetRailMode

from .clamp_logic import ClampDetector, OvertorqueDetector
from .swerve_math import calculate_swerve_commands


MODULE_NAMES = ('front_left', 'front_right', 'rear_right', 'rear_left')
FREE_SURFACE = 0
RAIL_ENTRY = 1
ON_RAIL = 2
RAIL_EXIT = 3


class RailControllerNode(Node):
    """Rail supervisor; hard current/torque protection remains on the STM32."""

    def __init__(self):
        super().__init__('rail_controller')
        self.declare_parameter('initial_mode', RAIL_ENTRY)
        self.declare_parameter('roller_radius', 0.075)
        self.declare_parameter('tire_radius', 0.0935)
        self.declare_parameter('module_x', [0.355, 0.355, -0.355, -0.355])
        self.declare_parameter('module_y', [0.335, -0.335, -0.335, 0.335])
        self.declare_parameter('steering_limit', 2.88)
        self.declare_parameter('max_wheel_angular_velocity', 8.0)
        self.declare_parameter('auto_start_probe', False)
        self.declare_parameter('initial_probe_mask', RailClampCommand.ALL_MODULES)
        self.declare_parameter('clamp_max_angle', math.radians(10.0))
        self.declare_parameter('clamp_angular_speed', math.radians(5.0))
        self.declare_parameter('clamp_directions', [-1.0, 1.0, 1.0, -1.0])
        self.declare_parameter('minimum_probe_angle', math.radians(0.5))
        self.declare_parameter('contact_effort_threshold', 1.0)
        self.declare_parameter('contact_velocity_threshold', 0.03)
        self.declare_parameter('contact_confirm_samples', 5)
        self.declare_parameter('hard_effort_limit', 3.0)
        self.declare_parameter('overtorque_confirm_samples', 2)
        self.declare_parameter('clamp_timeout', 5.0)
        self.declare_parameter('limit_settle_time', 0.3)
        self.declare_parameter('command_timeout', 0.25)
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('simulate_contact', False)
        self.declare_parameter(
            'simulated_contact_angles', [math.radians(2.0)] * 4)

        self.roller_radius = float(self.get_parameter('roller_radius').value)
        self.tire_radius = float(self.get_parameter('tire_radius').value)
        self.module_x = tuple(float(value) for value in self.get_parameter('module_x').value)
        self.module_y = tuple(float(value) for value in self.get_parameter('module_y').value)
        self.steering_limit = float(self.get_parameter('steering_limit').value)
        self.max_wheel_angular_velocity = float(
            self.get_parameter('max_wheel_angular_velocity').value)
        self.auto_start_probe = bool(self.get_parameter('auto_start_probe').value)
        self.initial_probe_mask = int(self.get_parameter('initial_probe_mask').value)
        self.max_angle = float(self.get_parameter('clamp_max_angle').value)
        self.clamp_speed = float(self.get_parameter('clamp_angular_speed').value)
        self.clamp_timeout = float(self.get_parameter('clamp_timeout').value)
        self.limit_settle_time = float(self.get_parameter('limit_settle_time').value)
        self.command_timeout = float(self.get_parameter('command_timeout').value)
        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.simulate_contact = bool(self.get_parameter('simulate_contact').value)
        self.clamp_directions = tuple(
            float(value) for value in self.get_parameter('clamp_directions').value)
        self.simulated_contact_angles = tuple(
            float(value) for value in self.get_parameter('simulated_contact_angles').value)
        if (
                len(self.clamp_directions) != 4
                or len(self.simulated_contact_angles) != 4
                or len(self.module_x) != 4 or len(self.module_y) != 4):
            raise ValueError('clamp direction and simulated contact lists must be FL,FR,RR,RL')
        if min(
            self.roller_radius, self.tire_radius, self.steering_limit,
            self.max_wheel_angular_velocity, self.max_angle, self.clamp_speed,
            self.clamp_timeout, self.command_timeout, self.publish_rate,
        ) <= 0.0 or self.limit_settle_time < 0.0:
            raise ValueError('rail controller radii, limits, timeouts, and rates are invalid')

        self.contact_effort_threshold = float(
            self.get_parameter('contact_effort_threshold').value)
        self.detector = ClampDetector(
            effort_threshold=self.contact_effort_threshold,
            velocity_threshold=float(self.get_parameter('contact_velocity_threshold').value),
            minimum_probe_angle=float(self.get_parameter('minimum_probe_angle').value),
            confirmation_samples=int(self.get_parameter('contact_confirm_samples').value),
        )
        self.overtorque_detector = OvertorqueDetector(
            effort_limit=float(self.get_parameter('hard_effort_limit').value),
            confirmation_samples=int(self.get_parameter('overtorque_confirm_samples').value),
        )

        self.positions = [0.0] * 4
        self.velocities = [0.0] * 4
        self.efforts = [0.0] * 4
        self.have_joint_state = [False] * 4
        self.have_effort_state = [False] * 4
        self.hold_angles = [0.0] * 4
        self.contact_flags = [False] * 4
        self.active_mode = int(self.get_parameter('initial_mode').value)
        self.active_module_mask = 0
        self.clamp_started_at = self.get_clock().now()
        self.last_command_at = self.get_clock().now()
        self.command_vx = 0.0
        self.command_vy = 0.0
        self.command_wz = 0.0
        self.failed_reason = ''
        self.feedback_state = RailClampFeedback.IDLE
        self.sequence_id = 0
        self.probe_armed = False
        self.probe_active = False
        self.release_active = False
        self.front_verified_latched = False

        self.steering_publisher = self.create_publisher(
            Float64MultiArray, 'steering_position_controller/commands', 10)
        self.wheel_publisher = self.create_publisher(
            Float64MultiArray, 'wheel_velocity_controller/commands', 10)
        self.engagement_publisher = self.create_publisher(
            RailEngagementState, 'rail_engagement_state', 10)
        semantic_qos = QoSProfile(depth=1)
        semantic_qos.reliability = ReliabilityPolicy.RELIABLE
        semantic_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.clamp_command_publisher = self.create_publisher(
            RailClampCommand, 'rail_clamp_command', semantic_qos)
        self.clamp_feedback_publisher = self.create_publisher(
            RailClampFeedback, 'rail_clamp_feedback', 10)
        self.create_subscription(JointState, 'joint_states', self.on_joint_state, 10)
        self.create_subscription(Twist, 'cmd_vel', self.on_command, 10)
        self.create_subscription(
            RailClampCommand, 'rail_clamp_request', self.on_clamp_request, 10)
        self.create_service(SetRailMode, 'set_rail_mode', self.on_set_mode)
        self.timer = self.create_timer(1.0 / self.publish_rate, self.on_timer)

        if self.active_mode == RAIL_ENTRY and self.auto_start_probe:
            self.start_clamp(self.initial_probe_mask)
        self.get_logger().info(
            f'Rail controller ready; clamp signs FL,FR,RR,RL={self.clamp_directions}, '
            f'max={math.degrees(self.max_angle):.1f} deg, simulated_contact={self.simulate_contact}'
        )

    @staticmethod
    def selected_indices(module_mask):
        return [index for index in range(4) if module_mask & (1 << index)]

    def next_sequence(self, requested_sequence=0):
        self.sequence_id = (
            int(requested_sequence) if requested_sequence else self.sequence_id + 1)

    def start_clamp(self, module_mask, requested_sequence=0):
        module_mask &= RailClampCommand.ALL_MODULES
        if not module_mask:
            module_mask = RailClampCommand.ALL_MODULES
        self.next_sequence(requested_sequence)
        self.active_mode = RAIL_ENTRY
        self.active_module_mask = module_mask
        self.clamp_started_at = self.get_clock().now()
        self.detector.reset()
        self.overtorque_detector.reset()
        self.contact_flags = [False] * 4
        self.hold_angles = list(self.positions)
        self.failed_reason = ''
        self.feedback_state = RailClampFeedback.PROBING
        self.probe_armed = all(self.have_joint_state)
        self.probe_active = True
        self.release_active = False
        if module_mask == RailClampCommand.FRONT_PAIR:
            self.front_verified_latched = False
        self.publish_semantic_command(RailClampCommand.START_PROBE, module_mask)

    def start_release(self, module_mask, requested_sequence=0):
        module_mask &= RailClampCommand.ALL_MODULES
        if not module_mask:
            module_mask = RailClampCommand.ALL_MODULES
        self.next_sequence(requested_sequence)
        self.active_module_mask = module_mask
        self.probe_active = False
        self.release_active = True
        self.failed_reason = ''
        self.feedback_state = RailClampFeedback.RELEASING
        self.publish_semantic_command(RailClampCommand.RELEASE, module_mask)

    def on_clamp_request(self, message):
        if message.action == RailClampCommand.START_PROBE:
            self.start_clamp(message.module_mask, message.sequence_id)
        elif message.action == RailClampCommand.RELEASE:
            self.start_release(message.module_mask, message.sequence_id)
        elif message.action == RailClampCommand.CANCEL:
            self.next_sequence(message.sequence_id)
            self.active_module_mask = message.module_mask & RailClampCommand.ALL_MODULES
            self.probe_active = False
            self.release_active = False
            self.feedback_state = RailClampFeedback.CANCELLED
            self.hold_angles = list(self.positions)
            self.publish_semantic_command(
                RailClampCommand.CANCEL, self.active_module_mask)

    def on_joint_state(self, message):
        index_by_name = {name: index for index, name in enumerate(message.name)}
        for module_index, module_name in enumerate(MODULE_NAMES):
            joint_index = index_by_name.get(module_name + '_steering_joint')
            if joint_index is None:
                continue
            if joint_index < len(message.position):
                self.positions[module_index] = message.position[joint_index]
                self.have_joint_state[module_index] = True
            if joint_index < len(message.velocity):
                self.velocities[module_index] = message.velocity[joint_index]
            if joint_index < len(message.effort):
                self.efforts[module_index] = message.effort[joint_index]
                self.have_effort_state[module_index] = True

    def on_command(self, message):
        values = (message.linear.x, message.linear.y, message.angular.z)
        if not all(math.isfinite(value) for value in values):
            self.get_logger().error('rejected non-finite cmd_vel')
            return
        self.command_vx, self.command_vy, self.command_wz = map(float, values)
        self.last_command_at = self.get_clock().now()

    def on_set_mode(self, request, response):
        requested = int(request.requested_mode)
        if requested not in (FREE_SURFACE, RAIL_ENTRY, ON_RAIL, RAIL_EXIT):
            response.accepted = False
            response.active_mode = self.active_mode
            response.message = 'unknown rail mode'
            return response
        if requested == RAIL_ENTRY:
            self.active_mode = RAIL_ENTRY
            self.failed_reason = ''
            self.feedback_state = RailClampFeedback.IDLE
            self.probe_active = False
            self.release_active = False
        elif requested == ON_RAIL and not all(self.contact_flags):
            response.accepted = False
            response.active_mode = self.active_mode
            response.message = 'ON_RAIL rejected until four contacts are confirmed'
            return response
        elif requested == RAIL_EXIT:
            self.active_mode = RAIL_EXIT
            self.start_release(RailClampCommand.ALL_MODULES)
        else:
            self.active_mode = requested
            if requested == FREE_SURFACE:
                self.hold_angles = [0.0] * 4
                self.contact_flags = [False] * 4
                self.feedback_state = RailClampFeedback.IDLE
                self.probe_active = False
                self.release_active = False
                self.front_verified_latched = False
                self.publish_semantic_command(
                    RailClampCommand.RELEASE, RailClampCommand.ALL_MODULES)
        response.accepted = True
        response.active_mode = self.active_mode
        response.message = 'mode accepted'
        return response

    def publish_semantic_command(self, action, module_mask):
        message = RailClampCommand()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'base_link'
        message.sequence_id = self.sequence_id
        message.action = action
        message.module_mask = module_mask
        message.max_probe_angle = self.max_angle
        message.probe_angular_speed = self.clamp_speed
        message.timeout = self.clamp_timeout
        self.clamp_command_publisher.publish(message)

    def publish_commands(self, steering, wheel):
        steering_message = Float64MultiArray()
        steering_message.data = list(steering)
        wheel_message = Float64MultiArray()
        wheel_message.data = list(wheel)
        self.steering_publisher.publish(steering_message)
        self.wheel_publisher.publish(wheel_message)

    def publish_state(self, engagement_state, reason):
        engagement = RailEngagementState()
        engagement.header.stamp = self.get_clock().now().to_msg()
        engagement.header.frame_id = 'base_link'
        engagement.state = engagement_state
        engagement.confidence = sum(self.contact_flags) / 4.0
        engagement.steering_torque = [float(value) for value in self.efforts]
        engagement.steering_angle_error = [
            float(target - actual) for target, actual in zip(self.hold_angles, self.positions)
        ]
        engagement.reason = reason
        self.engagement_publisher.publish(engagement)

        feedback = RailClampFeedback()
        feedback.header = engagement.header
        feedback.sequence_id = self.sequence_id
        feedback.state = self.feedback_state
        feedback.active_module_mask = self.active_module_mask
        feedback.contact_confirmed = list(self.contact_flags)
        feedback.steering_angle = [float(value) for value in self.positions]
        feedback.steering_velocity = [float(value) for value in self.velocities]
        feedback.steering_torque = [float(value) for value in self.efforts]
        feedback.reason = reason
        self.clamp_feedback_publisher.publish(feedback)

    def simulated_detector_inputs(self):
        velocities = list(self.velocities)
        efforts = list(self.efforts)
        if not self.simulate_contact:
            return velocities, efforts
        for index, angle in enumerate(self.positions):
            if abs(angle) >= self.simulated_contact_angles[index]:
                velocities[index] = 0.0
                efforts[index] = self.contact_effort_threshold * 1.2
        return velocities, efforts

    def fail_clamp(self, reason, state):
        self.failed_reason = reason
        self.feedback_state = state
        self.probe_active = False
        self.hold_angles = list(self.positions)
        self.publish_semantic_command(RailClampCommand.CANCEL, self.active_module_mask)
        self.get_logger().error(reason)

    def update_entry(self):
        selected = self.selected_indices(self.active_module_mask)
        elapsed = (self.get_clock().now() - self.clamp_started_at).nanoseconds / 1e9
        if not self.simulate_contact and not all(self.have_effort_state[i] for i in selected):
            if elapsed > 0.5:
                self.fail_clamp(
                    'steering effort feedback missing during clamp probe',
                    RailClampFeedback.SENSOR_FAULT)
            return

        overtorque = self.overtorque_detector.update(self.efforts)
        if any(overtorque[i] for i in selected):
            names = ','.join(MODULE_NAMES[i] for i in selected if overtorque[i])
            self.fail_clamp(
                f'hard steering torque limit exceeded: {names}',
                RailClampFeedback.OVERTORQUE)
            return

        detector_velocities, detector_efforts = self.simulated_detector_inputs()
        previous = list(self.contact_flags)
        detected = self.detector.update(
            self.positions, detector_velocities, detector_efforts)
        self.contact_flags = [
            detected[index] if index in selected else False for index in range(4)]
        for index in selected:
            if self.contact_flags[index] and not previous[index]:
                self.hold_angles[index] = self.positions[index]
                self.get_logger().info(
                    f'{MODULE_NAMES[index]} clamp contact at '
                    f'{math.degrees(self.positions[index]):.2f} deg')

        if all(self.contact_flags[index] for index in selected):
            self.probe_active = False
            self.feedback_state = RailClampFeedback.PROBE_CONFIRMED
            self.hold_angles = list(self.positions)
            if self.active_module_mask == RailClampCommand.FRONT_PAIR:
                self.front_verified_latched = True
                self.get_logger().info('Front steering modules confirmed rail contact')
            elif self.active_module_mask == RailClampCommand.ALL_MODULES:
                self.active_mode = ON_RAIL
                self.get_logger().info('Four steering modules confirmed rail contact')
            return

        self.feedback_state = (
            RailClampFeedback.PARTIAL_CONTACT
            if any(self.contact_flags[index] for index in selected)
            else RailClampFeedback.PROBING)
        probe_angle = min(self.max_angle, elapsed * self.clamp_speed)
        for index in selected:
            if not self.contact_flags[index]:
                self.hold_angles[index] = self.clamp_directions[index] * probe_angle

        limit_time = self.max_angle / self.clamp_speed + self.limit_settle_time
        if elapsed >= self.clamp_timeout or elapsed >= limit_time:
            missing = ','.join(
                MODULE_NAMES[i] for i in selected if not self.contact_flags[i])
            self.fail_clamp(
                f'clamp ended without requested contacts: {missing}',
                RailClampFeedback.TIMEOUT)

    def update_release(self):
        selected = self.selected_indices(self.active_module_mask)
        maximum_step = self.clamp_speed / self.publish_rate
        for index in selected:
            angle = self.positions[index]
            self.hold_angles[index] = (
                0.0 if abs(angle) <= maximum_step
                else angle - math.copysign(maximum_step, angle))
        if max(abs(self.positions[index]) for index in selected) <= math.radians(0.5):
            for index in selected:
                self.hold_angles[index] = 0.0
                self.contact_flags[index] = False
            self.release_active = False
            self.feedback_state = RailClampFeedback.IDLE
            if (
                    self.active_mode == RAIL_EXIT
                    and self.active_module_mask == RailClampCommand.ALL_MODULES):
                self.active_mode = FREE_SURFACE
                self.front_verified_latched = False

    def on_timer(self):
        if not all(self.have_joint_state):
            self.publish_commands(self.positions, [0.0] * 4)
            self.publish_state(RailEngagementState.UNKNOWN, 'waiting for four steering joints')
            return

        if self.probe_active and not self.probe_armed:
            self.clamp_started_at = self.get_clock().now()
            self.hold_angles = list(self.positions)
            self.detector.reset()
            self.overtorque_detector.reset()
            self.probe_armed = True

        if self.probe_active and not self.failed_reason:
            self.update_entry()
            self.publish_commands(self.hold_angles, [0.0] * 4)
            if self.failed_reason:
                state = RailEngagementState.OBSTRUCTED
                reason = self.failed_reason
            elif self.active_mode == ON_RAIL:
                state = RailEngagementState.FULLY_ENGAGED
                reason = 'four independent clamp contacts confirmed'
            elif self.feedback_state == RailClampFeedback.PROBE_CONFIRMED:
                state = RailEngagementState.FRONT_VERIFIED
                reason = 'front pair independently confirmed; final proof still required'
            else:
                state = RailEngagementState.VERIFYING
                reason = 'probing requested steering modules independently'
            self.publish_state(state, reason)
            return

        if self.active_mode == RAIL_ENTRY and self.failed_reason:
            self.publish_commands(self.hold_angles, [0.0] * 4)
            self.publish_state(RailEngagementState.OBSTRUCTED, self.failed_reason)
            return

        if self.release_active:
            self.update_release()
            self.publish_commands(self.hold_angles, [0.0] * 4)
            state = (
                RailEngagementState.DISENGAGED
                if self.active_mode == FREE_SURFACE else RailEngagementState.VERIFYING)
            self.publish_state(state, 'releasing requested modules toward steering zero')
            return

        if self.active_mode == RAIL_ENTRY:
            expired = (
                self.get_clock().now() - self.last_command_at
            ).nanoseconds / 1e9 > self.command_timeout
            if expired:
                steering, wheels = list(self.positions), [0.0] * 4
            else:
                steering, wheels = calculate_swerve_commands(
                    self.module_x, self.module_y, self.tire_radius,
                    self.steering_limit, self.max_wheel_angular_velocity,
                    self.command_vx, self.command_vy, self.command_wz,
                    self.positions)
            self.hold_angles = list(steering)
            self.publish_commands(steering, wheels)
            state = (
                RailEngagementState.FRONT_VERIFIED
                if self.front_verified_latched else RailEngagementState.VERIFYING)
            reason = (
                'front pair verified; advancing rear pair under entry supervisor'
                if self.front_verified_latched else 'waiting for staged clamp request')
            self.publish_state(state, reason)
            return

        if self.active_mode == ON_RAIL:
            expired = (
                self.get_clock().now() - self.last_command_at
            ).nanoseconds / 1e9 > self.command_timeout
            vx = 0.0 if expired else self.command_vx
            wheel_speed = vx / self.roller_radius
            self.publish_commands(self.hold_angles, [wheel_speed] * 4)
            self.publish_state(
                RailEngagementState.FULLY_ENGAGED,
                'four independent clamp contacts confirmed')
            return

        self.publish_commands([0.0] * 4, [0.0] * 4)
        self.publish_state(RailEngagementState.DISENGAGED, 'free-surface steering centered')


def main():
    rclpy.init()
    node = RailControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
