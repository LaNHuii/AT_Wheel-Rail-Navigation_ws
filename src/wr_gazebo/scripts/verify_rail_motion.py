#!/usr/bin/env python3
"""验收轨内直行、低速倒车和禁止横移的运动约束。"""

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


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class RailMotionVerifier(Node):
    def __init__(self):
        super().__init__('verify_rail_motion')
        self.model_pose = None
        self.wheel_poses = {}
        self.create_subscription(ModelStates, '/gazebo/model_states', self.model_callback, 10)
        self.create_subscription(LinkStates, '/gazebo/link_states', self.link_callback, 10)
        self.command_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

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
            self.command_publisher.publish(Twist())
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.model_pose is not None and len(self.wheel_poses) == len(WHEEL_LINKS):
                return True
        return False

    def command_for(self, linear_x, duration):
        command = Twist()
        command.linear.x = linear_x
        end_time = time.monotonic() + duration
        while time.monotonic() < end_time:
            self.command_publisher.publish(command)
            rclpy.spin_once(self, timeout_sec=0.05)

    def snapshot(self):
        position = self.model_pose.position
        return {
            'x': position.x,
            'y': position.y,
            'z': position.z,
            'yaw': yaw_from_quaternion(self.model_pose.orientation),
            'clearances': {
                name: pose.position.z - TIRE_RADIUS
                for name, pose in self.wheel_poses.items()
            },
        }


def main():
    rclpy.init()
    node = RailMotionVerifier()
    failures = []
    try:
        if not node.wait_until_ready(20.0):
            print('FAIL: Gazebo model/link states were not available within 20 seconds.')
            return 1

        node.command_for(0.0, 2.0)
        start = node.snapshot()
        node.command_for(0.10, 5.0)
        moving_end = node.snapshot()
        node.command_for(0.0, 1.0)
        stopped_first = node.snapshot()
        node.command_for(0.0, 2.0)
        stopped_final = node.snapshot()

        travel_x = moving_end['x'] - start['x']
        lateral_error = moving_end['y'] - start['y']
        yaw_error = wrap_angle(moving_end['yaw'] - start['yaw'])
        stop_drift = math.hypot(
            stopped_final['x'] - stopped_first['x'],
            stopped_final['y'] - stopped_first['y'],
        )
        stop_yaw_drift = abs(wrap_angle(stopped_final['yaw'] - stopped_first['yaw']))

        print(f'commanded: vx=0.100 m/s for 5.0 s')
        print(f'measured travel: dx={travel_x:.4f} m, dy={lateral_error:.4f} m, '
              f'dyaw={yaw_error:.4f} rad')
        print(f'stop drift over 2 s: xy={stop_drift * 1000.0:.2f} mm, '
              f'yaw={stop_yaw_drift:.4f} rad')
        for name in WHEEL_LINKS:
            clearance = stopped_final['clearances'][name]
            print(f'{name} final tire-bottom clearance: {clearance * 1000.0:.2f} mm')

        if travel_x < 0.10:
            failures.append('forward command did not produce at least 0.10 m forward travel')
        if travel_x > 0.80:
            failures.append('forward travel is implausibly large for the test command')
        if abs(lateral_error) > 0.05:
            failures.append('lateral deviation exceeded 50 mm during rail travel')
        if abs(yaw_error) > 0.10:
            failures.append('yaw deviation exceeded 0.10 rad during rail travel')
        if stop_drift > 0.020 or stop_yaw_drift > 0.05:
            failures.append('chassis did not remain stopped on the rails')
        touching = [
            name for name, value in stopped_final['clearances'].items()
            if value < 0.0002
        ]
        if touching:
            failures.append('tire(s) touched the floor after travel: ' + ', '.join(touching))

        if failures:
            for failure in failures:
                print(f'FAIL: {failure}')
            return 1

        print('RAIL_MOTION_VERIFICATION_PASSED')
        return 0
    finally:
        node.command_publisher.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
