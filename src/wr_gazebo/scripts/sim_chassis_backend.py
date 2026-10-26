#!/usr/bin/env python3
"""人物跟随集成测试使用的仿真 STM32 底盘后端。

本节点只用于 Gazebo 流程验收，模拟底盘模式、控制权、轮速和主 IMU 反馈。
它不代表真实电机动力学、CAN 时延或 STM32 硬保护已经验证。
仿真发布频率可通过 publish_rate 参数调节。
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool
from wr_interfaces.msg import ChassisControlCommand, ChassisState


class SimChassisBackend(Node):
    def __init__(self):
        super().__init__('sim_chassis_backend')
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('boot_id', 1)
        self.mode = ChassisState.MODE_FREE_SURFACE
        self.source = ChassisState.SOURCE_AUTONOMOUS
        self.command = Twist()
        self.last_command_sequence = 0
        self.manual_override = False
        self.yaw = 0.0
        self.last_update = self.get_clock().now()
        self.state_publisher = self.create_publisher(ChassisState, 'chassis_state', 20)
        self.imu_publisher = self.create_publisher(Imu, 'imu/data_raw', 20)
        self.create_subscription(Twist, 'cmd_vel_chassis', self.on_velocity, 20)
        self.create_subscription(
            ChassisControlCommand, 'chassis_control_command', self.on_control, 20)
        self.create_subscription(Bool, 'sim/manual_override', self.on_manual_override, 10)
        rate = float(self.get_parameter('publish_rate').value)
        self.timer = self.create_timer(1.0 / rate, self.on_timer)

    def on_velocity(self, message):
        if self.source == ChassisState.SOURCE_AUTONOMOUS and not self.manual_override:
            self.command = message
        else:
            self.command = Twist()

    def on_control(self, message):
        self.last_command_sequence = int(message.sequence_id)
        if self.manual_override or message.requested_control_source != message.SOURCE_AUTONOMOUS:
            return
        if message.requested_mode in (
                message.MODE_FREE_SURFACE, message.MODE_RAIL_ENTRY,
                message.MODE_ON_RAIL, message.MODE_RAIL_EXIT):
            self.mode = int(message.requested_mode)
            self.source = ChassisState.SOURCE_AUTONOMOUS

    def on_manual_override(self, message):
        # 模拟遥控接管：接管与释放都立即清零速度，
        # 释放后保持 SOURCE_NONE，等待上位机重新申请自动控制。
        self.manual_override = bool(message.data)
        self.command = Twist()
        self.source = (
            ChassisState.SOURCE_MANUAL if self.manual_override
            else ChassisState.SOURCE_NONE)

    def on_timer(self):
        now = self.get_clock().now()
        dt = max(0.0, min(0.1, (now - self.last_update).nanoseconds * 1.0e-9))
        self.last_update = now
        self.yaw = math.remainder(self.yaw + self.command.angular.z * dt, 2.0 * math.pi)
        state = ChassisState()
        state.header.stamp = now.to_msg()
        state.header.frame_id = 'base_link'
        state.boot_id = int(self.get_parameter('boot_id').value)
        state.last_command_sequence_id = self.last_command_sequence
        state.active_control_source = self.source
        state.active_mode = self.mode
        state.enabled = True
        state.physical_estop = False
        state.remote_online = self.manual_override
        state.command_timed_out = False
        state.motor_power_ready = True
        state.measured_twist = self.command
        self.state_publisher.publish(state)
        imu = Imu()
        imu.header = state.header
        imu.header.frame_id = 'imu_link'
        imu.orientation.z = math.sin(0.5 * self.yaw)
        imu.orientation.w = math.cos(0.5 * self.yaw)
        imu.angular_velocity.z = self.command.angular.z
        imu.orientation_covariance[0] = 0.01
        imu.orientation_covariance[4] = 0.01
        imu.orientation_covariance[8] = 0.02
        imu.angular_velocity_covariance[0] = 0.0004
        imu.angular_velocity_covariance[4] = 0.0004
        imu.angular_velocity_covariance[8] = 0.0004
        imu.linear_acceleration_covariance[0] = 0.04
        imu.linear_acceleration_covariance[4] = 0.04
        imu.linear_acceleration_covariance[8] = 0.04
        self.imu_publisher.publish(imu)


def main(args=None):
    rclpy.init(args=args)
    node = SimChassisBackend()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
