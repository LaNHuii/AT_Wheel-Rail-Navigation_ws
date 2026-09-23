#!/usr/bin/env python3

import math
import sys
import time

import rclpy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist
from rclpy.node import Node


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


def wrapped_delta(after, before):
    return math.atan2(math.sin(after - before), math.cos(after - before))


class FlatGroundVerifier(Node):
    def __init__(self):
        super().__init__('flat_ground_verifier')
        self.pose = None
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(
            ModelStates, '/gazebo/model_states', self.on_model_states, 10
        )

    def on_model_states(self, message):
        try:
            index = message.name.index('wheel_rail_chassis')
        except ValueError:
            return
        self.pose = message.pose[index]

    def spin_for(self, seconds, command=None):
        deadline = time.monotonic() + seconds
        command = command or Twist()
        while time.monotonic() < deadline:
            self.publisher.publish(command)
            rclpy.spin_once(self, timeout_sec=0.04)

    def wait_for_model(self, timeout=20.0):
        deadline = time.monotonic() + timeout
        while self.pose is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.pose is None:
            raise RuntimeError('wheel_rail_chassis did not appear in /gazebo/model_states')

    def snapshot(self):
        p = self.pose.position
        roll, pitch, yaw = quaternion_to_rpy(self.pose.orientation)
        values = (p.x, p.y, p.z, roll, pitch, yaw)
        if not all(math.isfinite(value) for value in values):
            raise RuntimeError(f'non-finite model pose: {values}')
        return values


def displacement(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def main():
    rclpy.init()
    node = FlatGroundVerifier()
    try:
        node.wait_for_model()

        initial = node.snapshot()
        node.spin_for(3.0)
        settled = node.snapshot()
        require(displacement(initial, settled) < 0.08, f'excessive static drift: {initial} -> {settled}')
        require(abs(settled[3]) < 0.15 and abs(settled[4]) < 0.15, f'chassis not level: {settled}')
        require(-0.05 < settled[2] < 0.15, f'unexpected root height: {settled[2]:.3f} m')
        print(f'PASS settle pose={settled}', flush=True)

        forward = Twist()
        forward.linear.x = 0.15
        before_forward = node.snapshot()
        node.spin_for(3.0, forward)
        after_forward = node.snapshot()
        require(after_forward[0] - before_forward[0] > 0.15, f'forward displacement too small: {before_forward} -> {after_forward}')
        require(abs(after_forward[3]) < 0.20 and abs(after_forward[4]) < 0.20, f'unstable during forward motion: {after_forward}')
        print(f'PASS forward dx={after_forward[0] - before_forward[0]:.3f} m', flush=True)

        node.spin_for(0.8)
        lateral = Twist()
        lateral.linear.y = 0.15
        before_lateral = node.snapshot()
        node.spin_for(3.0, lateral)
        after_lateral = node.snapshot()
        require(after_lateral[1] - before_lateral[1] > 0.12, f'lateral displacement too small: {before_lateral} -> {after_lateral}')
        require(abs(after_lateral[3]) < 0.20 and abs(after_lateral[4]) < 0.20, f'unstable during lateral motion: {after_lateral}')
        print(f'PASS lateral dy={after_lateral[1] - before_lateral[1]:.3f} m', flush=True)

        node.spin_for(0.8)
        rotate = Twist()
        rotate.angular.z = 0.25
        before_rotate = node.snapshot()
        node.spin_for(4.0, rotate)
        after_rotate = node.snapshot()
        dyaw = wrapped_delta(after_rotate[5], before_rotate[5])
        require(dyaw > 0.30, f'yaw change too small: {dyaw:.3f} rad')
        require(abs(after_rotate[3]) < 0.20 and abs(after_rotate[4]) < 0.20, f'unstable during rotation: {after_rotate}')
        print(f'PASS rotate dyaw={dyaw:.3f} rad', flush=True)

        node.spin_for(0.8)
        before_stop = node.snapshot()
        node.spin_for(1.0)
        after_stop = node.snapshot()
        require(displacement(before_stop, after_stop) < 0.06, f'chassis did not stop: {before_stop} -> {after_stop}')
        print(f'PASS stop drift={displacement(before_stop, after_stop):.3f} m', flush=True)
        print('FLAT_GROUND_VERIFICATION_PASSED', flush=True)
        return 0
    except Exception as error:
        print(f'FLAT_GROUND_VERIFICATION_FAILED: {error}', file=sys.stderr, flush=True)
        return 1
    finally:
        node.publisher.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
