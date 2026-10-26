#!/usr/bin/env python3
"""轨道入口感知 ROS 2 节点。

节点将点云转换到 base_link，调用轨道几何检测，并发布入口位姿、
对准误差和可通行性。雷达外参在 wr_description 中维护，
检测阈值在 config/rail_perception.yaml 中调整。
"""

import math

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformException, TransformListener
from wr_interfaces.msg import RailEntryEstimate

from .rail_geometry import RailGeometryConfig, detect_rail_geometry


class RailEntryPerceptionNode(Node):
    """封装坐标变换、点云转换和轨道入口结果发布。"""
    def __init__(self):
        super().__init__('rail_entry_perception')
        defaults = RailGeometryConfig()
        for name in defaults.__dataclass_fields__:
            self.declare_parameter(name, getattr(defaults, name))
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('max_points', 80000)
        values = {}
        for name, field in defaults.__dataclass_fields__.items():
            value = self.get_parameter(name).value
            values[name] = int(value) if field.type is int else float(value)
        self.config = RailGeometryConfig(**values)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.max_points = int(self.get_parameter('max_points').value)
        self.tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.publisher = self.create_publisher(RailEntryEstimate, 'rail_entry_estimate', 10)
        self.create_subscription(PointCloud2, 'lidar/points', self.on_cloud, 10)

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
        return points @ rotation.T + np.array([
            translation.x, translation.y, translation.z])

    def publish_unknown(self, header, reason):
        output = RailEntryEstimate()
        output.header = header
        output.header.frame_id = self.base_frame
        output.traversability = RailEntryEstimate.UNKNOWN
        output.reason = reason
        self.publisher.publish(output)

    def on_cloud(self, message):
        try:
            points = self.transform_points(message)
        except (TransformException, ValueError, AssertionError) as error:
            self.publish_unknown(message.header, str(error))
            return
        result = detect_rail_geometry(points, self.config)
        if not result.detected:
            self.publish_unknown(message.header, result.reason)
            return
        output = RailEntryEstimate()
        output.header = message.header
        output.header.frame_id = self.base_frame
        output.entry_pose.pose.position.x = result.entry_x
        output.entry_pose.pose.position.y = result.entry_y
        output.entry_pose.pose.orientation.z = math.sin(0.5 * result.heading)
        output.entry_pose.pose.orientation.w = math.cos(0.5 * result.heading)
        output.entry_pose.covariance[0] = 0.01
        output.entry_pose.covariance[7] = 0.01
        output.entry_pose.covariance[35] = math.radians(3.0) ** 2
        output.rail_direction.x = math.cos(result.heading)
        output.rail_direction.y = math.sin(result.heading)
        output.rail_spacing = result.spacing
        output.visible_length = result.visible_length
        output.lateral_error = result.entry_y
        output.heading_error = result.heading
        output.confidence = result.confidence
        output.traversability = (
            RailEntryEstimate.BLOCKED if result.blocked else RailEntryEstimate.PASSABLE)
        output.reason = result.reason
        self.publisher.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = RailEntryPerceptionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
