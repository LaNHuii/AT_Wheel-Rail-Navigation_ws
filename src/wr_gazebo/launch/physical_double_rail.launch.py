"""启动含物理接触的双圆轨 Gazebo 场景。"""

import os
import sys

sys.path.insert(0, os.path.dirname(__file__))

from rail_simulation import generate_rail_launch


def generate_launch_description():
    return generate_rail_launch('physical_double_rail.world', spawn_x=0.0)
