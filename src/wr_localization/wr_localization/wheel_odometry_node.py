"""轮速里程计 ROS 2 节点。

节点从 ChassisState.measured_twist 积分得到 odom->base_link 位姿，
为本地跟随控制和作业轨迹记录提供连续里程。
协方差和 max_dt 在本节点的 ROS 参数中调节，融合开关位于 config/local_ekf.yaml。
"""

from math import cos, sin

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from wr_interfaces.msg import ChassisState

from .integrator import PlanarTwistIntegrator


class WheelOdometryNode(Node):
    def __init__(self) -> None:
        super().__init__('wheel_odometry')
        self.declare_parameter('odom_frame', 'odom_wheel')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('max_dt', 0.25)
        self.declare_parameter('position_variance', 0.04)
        self.declare_parameter('yaw_variance', 0.09)
        self.declare_parameter('linear_velocity_variance', 0.01)
        self.declare_parameter('angular_velocity_variance', 0.02)
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.integrator = PlanarTwistIntegrator(float(self.get_parameter('max_dt').value))
        self.publisher = self.create_publisher(Odometry, 'wheel/odometry', 20)
        self.subscription = self.create_subscription(
            ChassisState, 'chassis_state', self._on_chassis_state, 20)

    def _on_chassis_state(self, state: ChassisState) -> None:
        stamp = float(state.header.stamp.sec) + float(state.header.stamp.nanosec) * 1.0e-9
        pose = self.integrator.update(
            stamp,
            int(state.boot_id),
            float(state.measured_twist.linear.x),
            float(state.measured_twist.linear.y),
            float(state.measured_twist.angular.z),
        )
        message = Odometry()
        message.header = state.header
        message.header.frame_id = self.odom_frame
        message.child_frame_id = self.base_frame
        message.pose.pose.position.x = pose.x
        message.pose.pose.position.y = pose.y
        message.pose.pose.orientation.z = sin(0.5 * pose.yaw)
        message.pose.pose.orientation.w = cos(0.5 * pose.yaw)
        message.twist.twist = state.measured_twist
        position_variance = float(self.get_parameter('position_variance').value)
        yaw_variance = float(self.get_parameter('yaw_variance').value)
        linear_variance = float(self.get_parameter('linear_velocity_variance').value)
        angular_variance = float(self.get_parameter('angular_velocity_variance').value)
        message.pose.covariance[0] = position_variance
        message.pose.covariance[7] = position_variance
        message.pose.covariance[35] = yaw_variance
        message.twist.covariance[0] = linear_variance
        message.twist.covariance[7] = linear_variance
        message.twist.covariance[35] = angular_variance
        self.publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WheelOdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
