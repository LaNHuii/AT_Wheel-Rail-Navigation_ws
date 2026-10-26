#!/usr/bin/env python3
"""MID-360S 点云安全节点，输出停车、前方阻塞和自由路面绕行建议。

点云先通过 TF 转到 base_link，再交给几何模块判断。
安装外参在 wr_description 中维护；安全范围和阈值在 config/lidar_safety.yaml 中调整。
"""

import math
import time

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformException, TransformListener
from wr_interfaces.msg import (
    ChassisState,
    LocalSafetyState,
    TrackedPerson,
    TraversabilityState,
)

from .safety_geometry import SafetyConfig, evaluate_safety, filter_obstacle_points
from .traversability import TraversabilityConfig, build_local_traversability


class LidarSafetyNode(Node):
    """为跟随行为提供偏保守的点云安全判断。"""

    def __init__(self):
        super().__init__('lidar_safety')
        # 高度、走廊宽度、停车距离、绕行速度等均来自
        # config/lidar_safety.yaml。实车调参前先核对 base_link 与雷达外参。
        defaults = SafetyConfig()
        for name in (
                'min_obstacle_z', 'max_obstacle_z', 'robot_front_x',
                'self_min_x', 'self_max_x', 'self_half_width', 'self_min_z',
                'self_max_z', 'free_corridor_half_width', 'rail_corridor_half_width',
                'hard_stop_distance', 'free_block_distance', 'rail_block_distance',
                'rotation_radius', 'target_exclusion_radius', 'bypass_offset',
                'bypass_corridor_half_width', 'bypass_check_distance'):
            self.declare_parameter(name, getattr(defaults, name))
        self.declare_parameter('minimum_points', defaults.minimum_points)
        self.declare_parameter('target_timeout', 0.30)
        self.declare_parameter('max_points', 60000)
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('bypass_lateral_speed', 0.06)
        traversability_defaults = TraversabilityConfig()
        for name in traversability_defaults.__dataclass_fields__:
            self.declare_parameter(name, getattr(traversability_defaults, name))
        self.config = SafetyConfig(**{
            name: float(self.get_parameter(name).value)
            for name in (
                'min_obstacle_z', 'max_obstacle_z', 'robot_front_x',
                'self_min_x', 'self_max_x', 'self_half_width', 'self_min_z',
                'self_max_z', 'free_corridor_half_width', 'rail_corridor_half_width',
                'hard_stop_distance', 'free_block_distance', 'rail_block_distance',
                'rotation_radius', 'target_exclusion_radius', 'bypass_offset',
                'bypass_corridor_half_width', 'bypass_check_distance')
        }, minimum_points=int(self.get_parameter('minimum_points').value))
        self.target_timeout = float(self.get_parameter('target_timeout').value)
        self.max_points = int(self.get_parameter('max_points').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.bypass_lateral_speed = float(
            self.get_parameter('bypass_lateral_speed').value)
        traversability_values = {}
        for name, field in traversability_defaults.__dataclass_fields__.items():
            value = self.get_parameter(name).value
            traversability_values[name] = int(value) if field.type is int else float(value)
        self.traversability_config = TraversabilityConfig(**traversability_values)
        if self.target_timeout <= 0.0 or self.max_points <= 0 or self.config.minimum_points <= 0:
            raise ValueError('LiDAR safety timeouts, point limits, and counts must be positive')

        self.mode = ChassisState.MODE_FREE_SURFACE
        self.target = None
        self.target_received_at = float('-inf')
        self.tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.publisher = self.create_publisher(LocalSafetyState, 'local_safety_state', 10)
        self.traversability_publisher = self.create_publisher(
            TraversabilityState, 'local_traversability', 10)
        self.grid_publisher = self.create_publisher(
            OccupancyGrid, 'local_obstacle_grid', 10)
        self.create_subscription(PointCloud2, 'lidar/points', self.on_cloud, 10)
        self.create_subscription(ChassisState, 'chassis_state', self.on_chassis, 10)
        self.create_subscription(TrackedPerson, 'tracked_person', self.on_target, 10)
        self.get_logger().info('LiDAR safety layer ready; waiting for point cloud and TF')

    def on_chassis(self, message):
        self.mode = int(message.active_mode)

    def on_target(self, message):
        self.target = message
        self.target_received_at = time.monotonic()

    @staticmethod
    def quaternion_matrix(rotation):
        x, y, z, w = rotation.x, rotation.y, rotation.z, rotation.w
        norm = x * x + y * y + z * z + w * w
        if norm < 1.0e-12:
            return np.eye(3)
        scale = 2.0 / norm
        return np.array([
            [1.0 - scale * (y * y + z * z), scale * (x * y - z * w),
             scale * (x * z + y * w)],
            [scale * (x * y + z * w), 1.0 - scale * (x * x + z * z),
             scale * (y * z - x * w)],
            [scale * (x * z - y * w), scale * (y * z + x * w),
             1.0 - scale * (x * x + y * y)],
        ])

    def transform_points(self, message):
        fields = {field.name for field in message.fields}
        if not {'x', 'y', 'z'}.issubset(fields):
            raise ValueError('PointCloud2 does not contain x/y/z fields')
        structured = point_cloud2.read_points(
            message, field_names=['x', 'y', 'z'], skip_nans=True)
        points = np.column_stack((structured['x'], structured['y'], structured['z']))
        if points.shape[0] > self.max_points:
            stride = int(math.ceil(points.shape[0] / self.max_points))
            points = points[::stride]
        if message.header.frame_id == self.base_frame:
            return points
        transform = self.tf_buffer.lookup_transform(
            self.base_frame, message.header.frame_id,
            Time.from_msg(message.header.stamp), timeout=Duration(seconds=0.05))
        rotation = self.quaternion_matrix(transform.transform.rotation)
        translation = transform.transform.translation
        offset = np.array([translation.x, translation.y, translation.z])
        return points @ rotation.T + offset

    def publish_fault(self, message, reason):
        output = LocalSafetyState()
        output.header.stamp = self.get_clock().now().to_msg()
        output.header.frame_id = self.base_frame
        output.data_fresh = False
        output.emergency_stop = True
        output.forward_path_blocked = True
        output.bypass_active = False
        output.rotation_safe = False
        output.nearest_obstacle_distance = -1.0
        output.suggested_lateral_velocity = 0.0
        output.reason = reason
        self.publisher.publish(output)
        traversability = TraversabilityState()
        traversability.header = output.header
        traversability.data_fresh = False
        traversability.preferred_direction = TraversabilityState.DIRECTION_NONE
        traversability.reason = reason
        self.traversability_publisher.publish(traversability)
        self.get_logger().warn(f'LiDAR safety rejected cloud: {reason}')

    def publish_traversability(self, header, result):
        """发布局部通行裕度和只表达障碍证据的滚动栅格。"""
        output = TraversabilityState()
        output.header = header
        output.header.frame_id = self.base_frame
        output.data_fresh = True
        output.forward_clearance = float(result.forward_clearance)
        output.left_clearance = float(result.left_clearance)
        output.right_clearance = float(result.right_clearance)
        output.preferred_direction = int(result.preferred_direction)
        output.obstacle_point_count = int(result.obstacle_point_count)
        output.reason = 'local obstacle-grid clearance evaluated'
        self.traversability_publisher.publish(output)

        cfg = self.traversability_config
        grid = OccupancyGrid()
        grid.header = output.header
        grid.info.resolution = float(cfg.grid_resolution)
        grid.info.width = int(result.occupancy.shape[0])
        grid.info.height = int(result.occupancy.shape[1])
        grid.info.origin.position.x = float(cfg.grid_min_x)
        grid.info.origin.position.y = float(-cfg.grid_half_width)
        grid.info.origin.orientation.w = 1.0
        # 这里只发布障碍证据：没有射线清空证据的栅格保持 unknown，避免误报自由空间。
        data = np.full(result.occupancy.T.shape, -1, dtype=np.int8)
        data[result.occupancy.T] = 100
        grid.data = data.reshape(-1).astype(int).tolist()
        self.grid_publisher.publish(grid)

    def on_cloud(self, message):
        try:
            points = self.transform_points(message)
        except (TransformException, ValueError, AssertionError) as error:
            self.publish_fault(message, str(error))
            return
        target_xy = None
        if self.target is not None and (
                time.monotonic() - self.target_received_at <= self.target_timeout
                and self.target.tracking_state == TrackedPerson.TRACKED
                and self.target.header.frame_id == self.base_frame):
            target_xy = (float(self.target.position.x), float(self.target.position.y))
        result = evaluate_safety(points, self.mode, self.config, target_xy)
        obstacle_points = filter_obstacle_points(points, self.config)
        traversability = build_local_traversability(
            obstacle_points, self.traversability_config)
        output = LocalSafetyState()
        output.header = message.header
        output.header.frame_id = self.base_frame
        output.data_fresh = True
        output.emergency_stop = result.emergency_stop
        output.forward_path_blocked = result.forward_path_blocked
        output.bypass_active = result.bypass_direction != 0
        output.rotation_safe = result.rotation_safe
        output.nearest_obstacle_distance = (
            float(result.nearest_obstacle_distance)
            if math.isfinite(result.nearest_obstacle_distance) else -1.0)
        output.suggested_lateral_velocity = (
            float(result.bypass_direction) * self.bypass_lateral_speed)
        output.reason = 'point-cloud safety zones evaluated'
        self.publisher.publish(output)
        self.publish_traversability(message.header, traversability)


def main(args=None):
    rclpy.init(args=args)
    node = LidarSafetyNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
