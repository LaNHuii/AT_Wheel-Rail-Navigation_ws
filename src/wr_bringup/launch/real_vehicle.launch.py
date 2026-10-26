"""启动真车人物跟随、局部感知、进退轨、健康门控和CAN-FD底盘链。"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _share(package, *parts):
    return os.path.join(get_package_share_directory(package), *parts)


def generate_launch_description():
    lidar_topic = LaunchConfiguration('lidar_topic')
    require_vision = LaunchConfiguration('require_vision')
    model = _share('wr_description', 'urdf', 'wr_chassis.urdf.xacro')
    robot_description = ParameterValue(Command(['xacro ', model]), value_type=str)

    chassis_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            _share('wr_chassis_gateway', 'launch', 'real_backend_can.launch.py')))
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            _share('wr_localization', 'launch', 'localization.launch.py')))

    return LaunchDescription([
        DeclareLaunchArgument(
            'lidar_topic', default_value='/livox/lidar',
            description='MID-360S的PointCloud2话题。'),
        DeclareLaunchArgument(
            'require_vision', default_value='true',
            description='完整人物跟随时为true；仅雷达联调时可设为false。'),
        Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}], output='screen'),
        chassis_launch,
        localization_launch,
        Node(
            package='wr_lidar_safety', executable='lidar_safety_node',
            parameters=[_share('wr_lidar_safety', 'config', 'lidar_safety.yaml')],
            remappings=[('lidar/points', lidar_topic)], output='screen'),
        Node(
            package='wr_rail_perception', executable='rail_entry_perception',
            parameters=[_share('wr_rail_perception', 'config', 'rail_perception.yaml')],
            remappings=[('lidar/points', lidar_topic)], output='screen'),
        Node(
            package='wr_runtime_monitor', executable='runtime_monitor_node',
            parameters=[
                _share('wr_runtime_monitor', 'config', 'runtime_monitor.yaml'),
                {'require_vision': ParameterValue(require_vision, value_type=bool)},
            ],
            remappings=[('lidar/points', lidar_topic)], output='screen'),
        Node(
            package='wr_person_following', executable='person_follow_node',
            parameters=[_share('wr_person_following', 'config', 'person_following.yaml')],
            output='screen'),
        Node(
            package='wr_rail_controller', executable='rail_controller_node',
            parameters=[_share('wr_rail_controller', 'config', 'rail_controller.yaml')],
            remappings=[('cmd_vel', 'cmd_vel_chassis')], output='screen'),
        Node(
            package='wr_rail_controller', executable='rail_transition_node',
            output='screen'),
        Node(
            package='wr_task_manager', executable='wr_task_manager_node',
            parameters=[{'autonomy_enabled_at_startup': False}], output='screen'),
    ])
