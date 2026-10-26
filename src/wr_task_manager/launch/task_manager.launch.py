"""启动顶层行为树任务管理器。"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='wr_task_manager',
            executable='wr_task_manager_node',
            name='wr_task_manager',
            output='screen',
        ),
    ])
