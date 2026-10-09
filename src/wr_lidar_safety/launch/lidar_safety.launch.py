import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('wr_lidar_safety')
    parameters = os.path.join(package_share, 'config', 'lidar_safety.yaml')
    return LaunchDescription([
        Node(
            package='wr_lidar_safety',
            executable='lidar_safety_node',
            parameters=[parameters],
            output='screen',
        ),
    ])
