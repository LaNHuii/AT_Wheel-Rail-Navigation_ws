"""启动轨道入口点云感知节点并加载双轨几何参数。"""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('wr_rail_perception'),
        'config', 'rail_perception.yaml')
    return LaunchDescription([
        Node(
            package='wr_rail_perception',
            executable='rail_entry_perception',
            name='rail_entry_perception',
            parameters=[config],
            remappings=[('lidar/points', '/livox/lidar')],
            output='screen',
        ),
    ])
