"""启动使用 SocketCAN 的真车底盘网关。"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('wr_chassis_gateway')
    parameters = os.path.join(package_share, 'config', 'chassis_gateway_can.yaml')
    return LaunchDescription([
        Node(
            package='wr_chassis_gateway', executable='cmd_vel_arbiter_node',
            parameters=[parameters], output='screen'
        ),
        Node(
            package='wr_chassis_gateway', executable='cmd_vel_guard_node',
            parameters=[parameters], output='screen',
            remappings=[('cmd_vel', 'cmd_vel_selected')]
        ),
        Node(
            package='wr_chassis_gateway', executable='real_chassis_adapter_node',
            parameters=[parameters], output='screen'
        ),
    ])
