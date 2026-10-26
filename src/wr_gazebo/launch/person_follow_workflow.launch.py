"""启动人物跟随、进离轨、底盘仲裁与本地定位的集成仿真。

其中 rear_advance_distance 和 exit_distance 为缩短测试时间的仿真专用值，
实车必须使用 wr_rail_controller/config/rail_controller.yaml 中标定后的行程。
"""

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

sys.path.insert(0, os.path.dirname(__file__))
from rail_simulation import generate_rail_launch


def generate_launch_description():
    description = generate_rail_launch('rail_entry.world', spawn_x=-1.8)
    description.entities.pop()
    rail_config = os.path.join(
        get_package_share_directory('wr_rail_controller'),
        'config', 'rail_controller_sim.yaml')
    person_config = os.path.join(
        get_package_share_directory('wr_person_following'),
        'config', 'person_following.yaml')
    localization_launch = os.path.join(
        get_package_share_directory('wr_localization'),
        'launch', 'localization.launch.py')
    description.add_action(Node(
        package='wr_rail_controller',
        executable='rail_controller_node',
        parameters=[rail_config, {
            'initial_mode': 0,
            'auto_start_probe': False,
            'use_sim_time': True,
        }],
        remappings=[
            ('cmd_vel', '/cmd_vel_chassis'),
            ('steering_position_controller/commands', '/gazebo_raw/steering_commands'),
            ('wheel_velocity_controller/commands', '/gazebo_raw/wheel_commands'),
        ],
        output='screen',
    ))
    description.add_action(Node(
        package='wr_chassis_gateway',
        executable='cmd_vel_arbiter_node',
        parameters=[{'use_sim_time': True}],
        remappings=[('cmd_vel_selected', '/cmd_vel')],
        output='screen',
    ))
    description.add_action(Node(
        package='wr_gazebo',
        executable='sim_chassis_backend.py',
        parameters=[{'use_sim_time': True}],
        output='screen',
    ))
    description.add_action(Node(
        package='wr_person_following',
        executable='person_follow_node',
        parameters=[person_config, {'use_sim_time': True}],
        output='screen',
    ))
    description.add_action(Node(
        package='wr_rail_controller',
        executable='rail_transition_node',
        parameters=[{
            'use_sim_time': True,
            'rear_advance_distance': 0.12,
            'exit_distance': 0.15,
            'rear_advance_speed': 0.08,
            'exit_speed': 0.08,
        }],
        output='screen',
    ))
    description.add_action(Node(
        package='wr_task_manager',
        executable='wr_task_manager_node',
        parameters=[{'use_sim_time': True}],
        output='screen',
    ))
    description.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(localization_launch),
        launch_arguments={'use_sim_time': 'true'}.items(),
    ))
    return description
