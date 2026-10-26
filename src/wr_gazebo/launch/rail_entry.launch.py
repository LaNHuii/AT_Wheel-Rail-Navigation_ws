"""启动标准轨道入口仿真场景。"""

import os
import sys

sys.path.insert(0, os.path.dirname(__file__))

from rail_simulation import generate_rail_launch


def generate_launch_description():
    return generate_rail_launch('rail_entry.world', spawn_x=-1.8)
