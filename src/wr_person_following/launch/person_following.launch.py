import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('wr_person_following')
    parameters = os.path.join(package_share, 'config', 'person_following.yaml')
    return LaunchDescription([
        Node(
            package='wr_person_following',
            executable='person_follow_node',
            parameters=[parameters],
            output='screen',
        ),
    ])
