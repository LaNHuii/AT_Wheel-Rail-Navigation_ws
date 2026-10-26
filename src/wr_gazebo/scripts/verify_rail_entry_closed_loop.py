#!/usr/bin/env python3
"""验收轨道入口对准与进轨的闭环控制。"""

import math
import sys
import time

import rclpy
from gazebo_msgs.msg import LinkStates, ModelStates
from geometry_msgs.msg import Twist
from rclpy.node import Node


MODEL_NAME = 'wheel_rail_chassis'
WHEEL_LINKS = (
    'front_left_wheel_link',
    'front_right_wheel_link',
    'rear_right_wheel_link',
    'rear_left_wheel_link',
)
TIRE_RADIUS = 0.0935


def clamp(value, lower, upper):
    return max(lower, min(value, upper))


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


class ClosedLoopEntryVerifier(Node):
    def __init__(self):
        super().__init__('verify_rail_entry_closed_loop')
        self.model_pose = None
        self.wheel_poses = {}
        self.create_subscription(ModelStates, '/gazebo/model_states', self.model_callback, 10)
        self.create_subscription(LinkStates, '/gazebo/link_states', self.link_callback, 10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

    def model_callback(self, message):
        if MODEL_NAME in message.name:
            self.model_pose = message.pose[message.name.index(MODEL_NAME)]

    def link_callback(self, message):
        for short_name in WHEEL_LINKS:
            suffix = f'::{short_name}'
            for index, full_name in enumerate(message.name):
                if full_name.endswith(suffix):
                    self.wheel_poses[short_name] = message.pose[index]
                    break

    def wait_until_ready(self, timeout):
        end_time = time.monotonic() + timeout
        while time.monotonic() < end_time:
            self.publisher.publish(Twist())
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.model_pose is not None and len(self.wheel_poses) == len(WHEEL_LINKS):
                return True
        return False

    def snapshot(self):
        return {
            'x': self.model_pose.position.x,
            'y': self.model_pose.position.y,
            'z': self.model_pose.position.z,
            'yaw': yaw_from_quaternion(self.model_pose.orientation),
            'clearances': {
                name: pose.position.z - TIRE_RADIUS
                for name, pose in self.wheel_poses.items()
            },
        }

    def zero_for(self, duration):
        end_time = time.monotonic() + duration
        while time.monotonic() < end_time:
            self.publisher.publish(Twist())
            rclpy.spin_once(self, timeout_sec=0.05)

    def enter_with_truth_feedback(self, timeout):
        end_time = time.monotonic() + timeout
        lifted_samples = 0
        peak_abs_y = 0.0
        peak_abs_yaw = 0.0
        while time.monotonic() < end_time:
            rclpy.spin_once(self, timeout_sec=0.02)
            state = self.snapshot()
            peak_abs_y = max(peak_abs_y, abs(state['y']))
            peak_abs_yaw = max(peak_abs_yaw, abs(state['yaw']))

            command = Twist()
            command.linear.x = 0.10
            command.linear.y = clamp(-1.5 * state['y'], -0.035, 0.035)
            command.angular.z = clamp(-2.0 * state['yaw'], -0.12, 0.12)
            self.publisher.publish(command)

            if min(state['clearances'].values()) > 0.002:
                lifted_samples += 1
                if lifted_samples >= 10:
                    return True, peak_abs_y, peak_abs_yaw
            else:
                lifted_samples = 0
        return False, peak_abs_y, peak_abs_yaw


def main():
    rclpy.init()
    node = ClosedLoopEntryVerifier()
    failures = []
    try:
        if not node.wait_until_ready(20.0):
            print('FAIL: Gazebo model/link states were not available within 20 seconds.')
            return 1

        node.zero_for(2.0)
        start = node.snapshot()
        entered, peak_abs_y, peak_abs_yaw = node.enter_with_truth_feedback(35.0)
        entry_end = node.snapshot()
        node.zero_for(1.0)
        stopped_first = node.snapshot()
        node.zero_for(2.0)
        stopped_final = node.snapshot()

        travel = entry_end['x'] - start['x']
        stop_drift = math.hypot(
            stopped_final['x'] - stopped_first['x'],
            stopped_final['y'] - stopped_first['y'],
        )

        print('alignment source: Gazebo ground truth (test-only perception substitute)')
        print(f'start pose: x={start["x"]:.4f} m, y={start["y"]:.4f} m, '
              f'yaw={start["yaw"]:.4f} rad')
        print(f'entry travel: dx={travel:.4f} m, peak |y|={peak_abs_y:.4f} m, '
              f'peak |yaw|={peak_abs_yaw:.4f} rad')
        for name in WHEEL_LINKS:
            print(f'{name} final tire clearance: '
                  f'{stopped_final["clearances"][name] * 1000.0:.2f} mm')
        print(f'stop drift over 2 s: {stop_drift * 1000.0:.2f} mm')

        if max(start['clearances'].values()) > 0.001:
            failures.append('test did not start with all four tires on the concrete')
        if not entered:
            failures.append('all four tires did not lift within the 35 second entry window')
        if travel < 1.0:
            failures.append('chassis did not advance far enough through the entry')
        if peak_abs_y > 0.05:
            failures.append('closed-loop entry exceeded 50 mm lateral error')
        if peak_abs_yaw > 0.12:
            failures.append('closed-loop entry exceeded 0.12 rad yaw error')
        not_lifted = [
            name for name, value in stopped_final['clearances'].items()
            if value < 0.002
        ]
        if not_lifted:
            failures.append('tire(s) were not supported clear of the floor: ' + ', '.join(not_lifted))
        if stop_drift > 0.030:
            failures.append('chassis did not remain stopped after entry')

        if failures:
            for failure in failures:
                print(f'FAIL: {failure}')
            return 1

        print('RAIL_ENTRY_CLOSED_LOOP_VERIFICATION_PASSED')
        return 0
    finally:
        node.publisher.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
