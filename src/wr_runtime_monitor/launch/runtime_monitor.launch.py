"""启动真车传感器健康监测节点。"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('wr_runtime_monitor'), 'config', 'runtime_monitor.yaml')
    return LaunchDescription([
        Node(
            package='wr_runtime_monitor',
            executable='runtime_monitor_node',
            parameters=[config],
            output='screen',
        ),
    ])
