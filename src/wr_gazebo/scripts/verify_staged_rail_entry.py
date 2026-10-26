#!/usr/bin/env python3
"""验收分阶段进轨监督器与 Gazebo 关节控制的集成。"""

import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from wr_interfaces.msg import RailClampCommand, RailClampFeedback
from wr_rail_controller.transition_logic import FixedFrontStagedEntryPolicy

from verify_rail_entry_closed_loop import ClosedLoopEntryVerifier


ENTRY_X = -1.0


class StagedEntryVerifier(ClosedLoopEntryVerifier):
    def __init__(self):
        super().__init__()
        self.feedback = None
        self.sequence = 0
        self.request_publisher = self.create_publisher(
            RailClampCommand, '/rail_clamp_request', 10)
        self.create_subscription(
            RailClampFeedback, '/rail_clamp_feedback', self.on_feedback, 10)

    def on_feedback(self, message):
        self.feedback = message

    def publish_request(self, action, module_mask):
        self.sequence += 1
        message = RailClampCommand()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'base_link'
        message.sequence_id = self.sequence
        message.action = action
        message.module_mask = module_mask
        message.max_probe_angle = 0.174532925
        message.probe_angular_speed = 0.087266463
        message.timeout = 5.0
        self.request_publisher.publish(message)

    def run_staged_entry(self, timeout):
        policy = FixedFrontStagedEntryPolicy()
        observed_phases = []
        end_time = time.monotonic() + timeout
        while time.monotonic() < end_time:
            rclpy.spin_once(self, timeout_sec=0.02)
            state = self.snapshot()
            feedback_state = (
                self.feedback.state if self.feedback else RailClampFeedback.IDLE)
            feedback_mask = (
                self.feedback.active_module_mask if self.feedback else 0)
            contacts = (
                tuple(self.feedback.contact_confirmed)
                if self.feedback else (False, False, False, False))
            result = policy.update(
                state['x'] - ENTRY_X, state['y'], state['yaw'],
                feedback_state, feedback_mask, contacts)
            if not observed_phases or observed_phases[-1] != result.phase:
                observed_phases.append(result.phase)
                print(f'staged entry phase: {result.phase}')
            command = Twist()
            command.linear.x = result.vx
            command.linear.y = result.vy
            command.angular.z = result.wz
            self.publisher.publish(command)
            if result.clamp_action:
                self.publish_request(result.clamp_action, result.module_mask)
            if result.failed:
                return False, observed_phases, 'clamp feedback reported failure'
            if result.done:
                return True, observed_phases, ''
        return False, observed_phases, f'timeout in {policy.phase}'


def main():
    rclpy.init()
    node = StagedEntryVerifier()
    try:
        if not node.wait_until_ready(20.0):
            print('FAIL: Gazebo model/link states unavailable')
            return 1
        node.zero_for(1.0)
        succeeded, phases, reason = node.run_staged_entry(60.0)
        node.zero_for(1.0)
        state = node.snapshot()
        required = {
            'front_probe', 'front_release', 'rear_advance',
            'final_probe', 'confirmed',
        }
        failures = []
        if not succeeded:
            failures.append(reason)
        if not required.issubset(phases):
            failures.append('missing staged phases: ' + ', '.join(sorted(required - set(phases))))
        if min(state['clearances'].values()) <= 0.002:
            failures.append('not all four tyres are clear of the concrete after final proof')
        if failures:
            for failure in failures:
                print(f'FAIL: {failure}')
            return 1
        print('STAGED_FIXED_FRONT_RAIL_ENTRY_VERIFICATION_PASSED')
        return 0
    finally:
        node.publisher.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
