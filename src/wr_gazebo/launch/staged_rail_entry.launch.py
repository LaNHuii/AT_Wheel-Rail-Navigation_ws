"""启动分阶段夹轨与自然进轨仿真。"""

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

sys.path.insert(0, os.path.dirname(__file__))

from rail_simulation import generate_rail_launch


def generate_launch_description():
    description = generate_rail_launch('u_rail_entry.world', spawn_x=-1.8)
    # 替换自由路面控制器：分阶段上轨监督器在进轨期间拥有关节命令权。
    # 速度话题仍位于 cmd_vel_guard 安全约束之后。
    description.entities.pop()
    parameters = os.path.join(
        get_package_share_directory('wr_rail_controller'),
        'config', 'rail_controller_sim.yaml')
    description.add_action(Node(
        package='wr_rail_controller',
        executable='rail_controller_node',
        parameters=[parameters, {'auto_start_probe': False, 'initial_mode': 1}],
        remappings=[
            ('cmd_vel', '/cmd_vel_chassis'),
            ('joint_states', '/joint_states'),
            ('steering_position_controller/commands', '/gazebo_raw/steering_commands'),
            ('wheel_velocity_controller/commands', '/gazebo_raw/wheel_commands'),
        ],
        output='screen',
    ))
    return description
