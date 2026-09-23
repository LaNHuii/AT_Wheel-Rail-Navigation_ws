#!/usr/bin/env python3

import math
import sys
import time

import rclpy
from rclpy.node import Node
from wr_interfaces.msg import RailClampFeedback


class ClampVerifier(Node):
    def __init__(self):
        super().__init__('verify_rail_clamp')
        self.feedback = None
        self.create_subscription(
            RailClampFeedback, '/rail_clamp_feedback', self.on_feedback, 10)

    def on_feedback(self, message):
        self.feedback = message

    def wait_for_terminal_state(self, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.feedback is not None and self.feedback.state in (
                RailClampFeedback.ENGAGED,
                RailClampFeedback.TIMEOUT,
                RailClampFeedback.OVERTORQUE,
                RailClampFeedback.SENSOR_FAULT,
            ):
                return self.feedback
        return self.feedback


def main():
    rclpy.init()
    node = ClampVerifier()
    failures = []
    try:
        feedback = node.wait_for_terminal_state(12.0)
        if feedback is None:
            print('FAIL: no rail clamp feedback received')
            return 1
        angles = [math.degrees(value) for value in feedback.steering_angle]
        print(f'clamp state={feedback.state}, contacts={list(feedback.contact_confirmed)}')
        print('contact angles FL,FR,RR,RL deg: ' + ', '.join(f'{value:.2f}' for value in angles))
        print(f'reason: {feedback.reason}')
        if feedback.state != RailClampFeedback.ENGAGED:
            failures.append('clamp did not reach ENGAGED')
        if not all(feedback.contact_confirmed):
            failures.append('not all four module contacts were independently confirmed')
        if max(abs(value) for value in angles) > 10.25:
            failures.append('a steering module exceeded the 10 degree probe limit')
        if min(abs(value) for value in angles) < 0.5:
            failures.append('contact was accepted before the minimum probe angle')
        if failures:
            for failure in failures:
                print(f'FAIL: {failure}')
            return 1
        print('RAIL_CLAMP_VERIFICATION_PASSED')
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
