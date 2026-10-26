#!/usr/bin/env python3
"""验收车体、内侧滚动圆柱和双轨的静态尺寸与位姿。"""

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


def quaternion_to_rpy(q):
    sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)

    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


class StaticRailVerifier(Node):
    def __init__(self):
        super().__init__('verify_static_rail')
        self.model_pose = None
        self.wheel_poses = {}
        self.create_subscription(ModelStates, '/gazebo/model_states', self.model_callback, 10)
        self.create_subscription(LinkStates, '/gazebo/link_states', self.link_callback, 10)
        self.zero_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

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

    def spin_for(self, duration):
        end_time = time.monotonic() + duration
        zero = Twist()
        while time.monotonic() < end_time:
            self.zero_publisher.publish(zero)
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_until_ready(self, timeout):
        end_time = time.monotonic() + timeout
        while time.monotonic() < end_time:
            self.zero_publisher.publish(Twist())
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.model_pose is not None and len(self.wheel_poses) == len(WHEEL_LINKS):
                return True
        return False

    def snapshot(self):
        p = self.model_pose.position
        roll, pitch, yaw = quaternion_to_rpy(self.model_pose.orientation)
        clearances = {
            name: pose.position.z - TIRE_RADIUS
            for name, pose in self.wheel_poses.items()
        }
        return (p.x, p.y, p.z, roll, pitch, yaw, clearances)


def main():
    rclpy.init()
    node = StaticRailVerifier()
    failures = []
    try:
        if not node.wait_until_ready(20.0):
            print('FAIL: Gazebo model/link states were not available within 20 seconds.')
            return 1

        node.spin_for(3.0)
        first = node.snapshot()
        node.spin_for(3.0)
        final = node.snapshot()

        x0, y0, z0, _, _, yaw0, _ = first
        x1, y1, z1, roll1, pitch1, yaw1, clearances = final
        translation_drift = math.hypot(x1 - x0, y1 - y0)
        vertical_drift = abs(z1 - z0)
        yaw_drift = abs(wrap_angle(yaw1 - yaw0))

        print(f'base pose: x={x1:.4f} m, y={y1:.4f} m, z={z1:.4f} m')
        print(f'base attitude: roll={roll1:.4f} rad, pitch={pitch1:.4f} rad, yaw={yaw1:.4f} rad')
        for name in WHEEL_LINKS:
            print(f'{name} tire-bottom clearance: {clearances[name] * 1000.0:.2f} mm')
        print(f'settled drift over 3 s: xy={translation_drift * 1000.0:.2f} mm, '
              f'z={vertical_drift * 1000.0:.2f} mm, yaw={yaw_drift:.4f} rad')

        if max(abs(roll1), abs(pitch1)) > 0.10:
            failures.append('chassis roll/pitch exceeds 0.10 rad')
        if abs(y1) > 0.05:
            failures.append('chassis moved more than 50 mm away from rail centerline')
        if translation_drift > 0.02 or vertical_drift > 0.005 or yaw_drift > 0.05:
            failures.append('chassis did not remain statically settled')
        low_clearance = [name for name, value in clearances.items() if value < 0.0002]
        if low_clearance:
            failures.append('tire(s) are touching the floor: ' + ', '.join(low_clearance))
        if max(clearances.values()) - min(clearances.values()) > 0.010:
            failures.append('four-wheel support is uneven by more than 10 mm')
        if max(clearances.values()) > 0.040:
            failures.append('chassis is unrealistically high above the floor')

        if failures:
            for failure in failures:
                print(f'FAIL: {failure}')
            return 1

        print('RAIL_STATIC_VERIFICATION_PASSED')
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
