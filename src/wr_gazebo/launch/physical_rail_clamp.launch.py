import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

sys.path.insert(0, os.path.dirname(__file__))

from rail_simulation import generate_rail_launch


def generate_launch_description():
    description = generate_rail_launch('physical_double_rail.world', spawn_x=0.0)
    # generate_rail_launch appends the free-surface swerve controller last.
    # Rail clamp tests replace it, so only one node owns the joint command topics.
    description.entities.pop()
    parameters = os.path.join(
        get_package_share_directory('wr_rail_controller'), 'config', 'rail_controller_sim.yaml')
    description.add_action(Node(
        package='wr_rail_controller',
        executable='rail_controller_node',
        parameters=[parameters],
        remappings=[
            ('cmd_vel', '/cmd_vel_chassis'),
            ('joint_states', '/joint_states'),
            ('steering_position_controller/commands', '/gazebo_raw/steering_commands'),
            ('wheel_velocity_controller/commands', '/gazebo_raw/wheel_commands'),
        ],
        output='screen',
    ))
    return description
