"""启动已调整接触参数的双轨仿真场景。"""

import os
import xml.etree.ElementTree as ET

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


LEFT_STEERING_JOINTS = {'front_left_steering_joint', 'rear_left_steering_joint'}
RIGHT_STEERING_JOINTS = {'front_right_steering_joint', 'rear_right_steering_joint'}


def build_rail_robot_description(model, module_y_offset=0.046):
    root = ET.fromstring(xacro.process_file(model).toxml())
    for joint in root.findall('joint'):
        name = joint.attrib.get('name', '')
        if name not in LEFT_STEERING_JOINTS | RIGHT_STEERING_JOINTS:
            continue
        origin = joint.find('origin')
        xyz = [float(value) for value in origin.attrib['xyz'].split()]
        xyz[1] += module_y_offset if name in LEFT_STEERING_JOINTS else -module_y_offset
        origin.attrib['xyz'] = ' '.join(f'{value:.9f}' for value in xyz)
    return ET.tostring(root, encoding='unicode')


def generate_launch_description():
    gazebo_share = get_package_share_directory('gazebo_ros')
    simulation_share = get_package_share_directory('wr_gazebo')
    gateway_share = get_package_share_directory('wr_chassis_gateway')
    model = os.path.join(simulation_share, 'urdf', 'wr_chassis_gazebo.urdf.xacro')
    world = os.path.join(simulation_share, 'worlds', 'tuned_double_rail.world')
    controllers = os.path.join(simulation_share, 'config', 'gazebo_controllers.yaml')
    gateway_parameters = os.path.join(gateway_share, 'config', 'chassis_gateway.yaml')
    gui = LaunchConfiguration('gui')
    robot_description = build_rail_robot_description(model)

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_share, 'launch', 'gazebo.launch.py')),
        launch_arguments={'world': world, 'gui': gui, 'verbose': 'false', 'pause': 'false'}.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='false'),
        gazebo,
        Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
            output='screen',
        ),
        Node(
            package='gazebo_ros', executable='spawn_entity.py',
            arguments=['-entity', 'wheel_rail_chassis', '-topic', 'robot_description',
                       '-x', '0.0', '-y', '0.0', '-z', '0.05'],
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
            package='wr_chassis_gateway', executable='cmd_vel_guard_node',
            parameters=[gateway_parameters, {'use_sim_time': True}], output='screen',
        ),
        Node(
            package='wr_gazebo', executable='gazebo_command_adapter.py',
            parameters=[{
                'drive_signs': [-1.0, 1.0, -1.0, -1.0],
                'steering_signs': [1.0, 1.0, 1.0, 1.0],
                'steering_offsets': [0.0, 0.0, 0.0, 0.0],
            }], output='screen',
        ),
        Node(
            package='wr_swerve_controller', executable='swerve_controller_node',
            parameters=[controllers],
            remappings=[
                ('cmd_vel', '/cmd_vel_chassis'),
                ('joint_states', '/joint_states'),
                ('swerve_state', '/swerve_state'),
                ('steering_position_controller/commands', '/gazebo_raw/steering_commands'),
                ('wheel_velocity_controller/commands', '/gazebo_raw/wheel_commands'),
            ], output='screen',
        ),
    ])
