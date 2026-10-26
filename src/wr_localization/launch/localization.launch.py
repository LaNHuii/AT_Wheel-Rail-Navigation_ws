"""启动轮速里程计与底盘主 IMU 的平面 EKF 融合。"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    configuration = os.path.join(
        get_package_share_directory('wr_localization'), 'config', 'local_ekf.yaml')
    use_sim_time = LaunchConfiguration('use_sim_time')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='wr_localization',
            executable='wheel_odometry_node',
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            parameters=[configuration, {'use_sim_time': use_sim_time}],
            output='screen',
        ),
    ])
