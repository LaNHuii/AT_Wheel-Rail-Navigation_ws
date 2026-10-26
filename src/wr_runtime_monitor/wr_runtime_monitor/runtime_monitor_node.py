#!/usr/bin/env python3
"""汇总雷达、底盘IMU、底盘、局部安全、定位和视觉数据健康状态。

各数据超时和是否作为必需条件，在 ``config/runtime_monitor.yaml`` 中调节。
节点只发布健康结论，不取代各控制节点和 STM32 自身的看门狗。
"""

import math
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, PointCloud2
from wr_interfaces.msg import ChassisState, LocalSafetyState, SystemHealth, TrackedPerson

from .health_logic import HealthConfig, HealthInputs, evaluate_health


class RuntimeMonitorNode(Node):
    """以统一话题发布自主运行所需数据链的健康状态。"""

    def __init__(self):
        super().__init__('runtime_monitor')
        defaults = HealthConfig()
        for name in defaults.__dataclass_fields__:
            self.declare_parameter(name, getattr(defaults, name))
        self.declare_parameter('publish_rate', 10.0)
        values = {
            name: self.get_parameter(name).value
            for name in defaults.__dataclass_fields__
        }
        self.config = HealthConfig(**values)
        publish_rate = float(self.get_parameter('publish_rate').value)
        timeout_values = [
            self.config.lidar_timeout, self.config.imu_timeout,
            self.config.chassis_timeout, self.config.safety_timeout,
            self.config.localization_timeout, self.config.vision_timeout,
        ]
        if publish_rate <= 0.0 or not all(math.isfinite(v) and v > 0.0 for v in timeout_values):
            raise ValueError('运行监测频率和超时参数必须为正数')

        self.received_at = {
            name: float('-inf')
            for name in ('lidar', 'imu', 'chassis', 'safety', 'localization', 'vision')
        }
        self.chassis_safe = False
        self.safety_stop = True
        self.publisher = self.create_publisher(SystemHealth, 'system_health', 10)
        self.create_subscription(
            PointCloud2, 'lidar/points', lambda _: self._touch('lidar'),
            qos_profile_sensor_data)
        self.create_subscription(
            Imu, 'imu/data_raw', lambda _: self._touch('imu'), qos_profile_sensor_data)
        self.create_subscription(ChassisState, 'chassis_state', self._on_chassis, 10)
        self.create_subscription(LocalSafetyState, 'local_safety_state', self._on_safety, 10)
        self.create_subscription(
            Odometry, 'odometry/filtered', lambda _: self._touch('localization'), 10)
        self.create_subscription(TrackedPerson, 'tracked_person', lambda _: self._touch('vision'), 10)
        self.timer = self.create_timer(1.0 / publish_rate, self._on_timer)

    def _touch(self, name):
        self.received_at[name] = time.monotonic()

    def _on_chassis(self, message):
        self._touch('chassis')
        self.chassis_safe = bool(
            message.enabled and message.motor_power_ready and not message.physical_estop
            and not message.command_timed_out and not message.chassis_fault_bits)

    def _on_safety(self, message):
        self._touch('safety')
        self.safety_stop = bool(
            not message.data_fresh or message.emergency_stop)

    def _on_timer(self):
        result = evaluate_health(HealthInputs(
            now=time.monotonic(),
            lidar_at=self.received_at['lidar'],
            imu_at=self.received_at['imu'],
            chassis_at=self.received_at['chassis'],
            safety_at=self.received_at['safety'],
            localization_at=self.received_at['localization'],
            vision_at=self.received_at['vision'],
            chassis_safe=self.chassis_safe,
            safety_stop=self.safety_stop,
        ), self.config)
        message = SystemHealth()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'base_link'
        message.lidar_fresh = result.lidar_fresh
        message.imu_fresh = result.imu_fresh
        message.chassis_fresh = result.chassis_fresh
        message.safety_fresh = result.safety_fresh
        message.localization_fresh = result.localization_fresh
        message.vision_fresh = result.vision_fresh
        message.autonomy_ready = result.autonomy_ready
        message.fault_mask = result.fault_mask
        message.reason = result.reason
        self.publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = RuntimeMonitorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
