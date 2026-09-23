#!/usr/bin/env python3

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
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


class RailEntryVerifier(Node):
    def __init__(self):
        super().__init__('verify_rail_entry')
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

    def publish_for(self, speed, duration):
        message = Twist()
        message.linear.x = speed
        end_time = time.monotonic() + duration
        while time.monotonic() < end_time:
            self.publisher.publish(message)
            rclpy.spin_once(self, timeout_sec=0.05)

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


def main():
    rclpy.init()
    node = RailEntryVerifier()
    failures = []
    try:
        if not node.wait_until_ready(20.0):
            print('FAIL: Gazebo model/link states were not available within 20 seconds.')
            return 1

        node.publish_for(0.0, 2.0)
        start = node.snapshot()
        node.publish_for(0.10, 25.0)
        entered = node.snapshot()
        node.publish_for(0.0, 1.0)
        stopped_first = node.snapshot()
        node.publish_for(0.0, 2.0)
        stopped_final = node.snapshot()

        travel = entered['x'] - start['x']
        lateral_error = entered['y'] - start['y']
        yaw_error = wrap_angle(entered['yaw'] - start['yaw'])
        stop_drift = math.hypot(
            stopped_final['x'] - stopped_first['x'],
            stopped_final['y'] - stopped_first['y'],
        )

        print(f'start pose: x={start["x"]:.4f} m, y={start["y"]:.4f} m, z={start["z"]:.4f} m')
        for name in WHEEL_LINKS:
            print(f'{name} initial tire clearance: {start["clearances"][name] * 1000.0:.2f} mm')
        print(f'entry travel: dx={travel:.4f} m, dy={lateral_error:.4f} m, '
              f'dyaw={yaw_error:.4f} rad')
        for name in WHEEL_LINKS:
            print(f'{name} final tire clearance: '
                  f'{stopped_final["clearances"][name] * 1000.0:.2f} mm')
        print(f'stop drift over 2 s: {stop_drift * 1000.0:.2f} mm')

        if max(start['clearances'].values()) > 0.001:
            failures.append('test did not start with all four tires on the concrete')
        if travel < 1.10:
            failures.append('chassis did not advance far enough to place all rollers on the rails')
        if abs(lateral_error) > 0.05:
            failures.append('lateral entry error exceeded 50 mm')
        if abs(yaw_error) > 0.10:
            failures.append('entry yaw error exceeded 0.10 rad')
        not_lifted = [
            name for name, value in stopped_final['clearances'].items()
            if value < 0.002
        ]
        if not_lifted:
            failures.append('tire(s) did not lift after entry: ' + ', '.join(not_lifted))
        if stop_drift > 0.030:
            failures.append('chassis did not remain stopped after entry')

        if failures:
            for failure in failures:
                print(f'FAIL: {failure}')
            return 1

        print('RAIL_ENTRY_VERIFICATION_PASSED')
        return 0
    finally:
        node.publisher.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
