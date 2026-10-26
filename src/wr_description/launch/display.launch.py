"""启动 robot_state_publisher 和 RViz，用于检查车体模型、关节和传感器坐标系。"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory('wr_description')
    model = os.path.join(package_share, 'urdf', 'wr_chassis.urdf.xacro')
    use_gui = LaunchConfiguration('use_gui')
    use_rviz = LaunchConfiguration('use_rviz')
    robot_description = ParameterValue(Command(['xacro ', model]), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_gui', default_value='true',
            description='Use the graphical joint-state publisher.'
        ),
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='Start RViz2.'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen',
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            condition=IfCondition(use_gui),
            output='screen',
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            condition=UnlessCondition(use_gui),
            output='screen',
        ),
        Node(
            package='rviz2', executable='rviz2',
            condition=IfCondition(use_rviz), output='screen'
        ),
    ])
