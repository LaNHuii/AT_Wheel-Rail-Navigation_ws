"""启动舍轮解算与 mock ros2_control 控制器，用于无真车时的接口检查。"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    description_share = get_package_share_directory('wr_description')
    controller_share = get_package_share_directory('wr_swerve_controller')
    model = os.path.join(
        description_share, 'urdf', 'wr_chassis_mock_control.urdf.xacro'
    )
    controllers = os.path.join(controller_share, 'config', 'mock_controllers.yaml')
    robot_description = ParameterValue(Command(['xacro ', model]), value_type=str)

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[{'robot_description': robot_description}, controllers],
            output='screen',
        ),
        Node(
            package='controller_manager', executable='spawner',
            arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
            output='screen',
        ),
        Node(
            package='controller_manager', executable='spawner',
            arguments=['steering_position_controller', '--controller-manager', '/controller_manager'],
            output='screen',
        ),
        Node(
            package='controller_manager', executable='spawner',
            arguments=['wheel_velocity_controller', '--controller-manager', '/controller_manager'],
            output='screen',
        ),
        Node(
            package='wr_swerve_controller', executable='swerve_controller_node',
            parameters=[controllers], output='screen',
        ),
    ])
