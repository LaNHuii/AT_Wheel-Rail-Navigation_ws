#!/usr/bin/env bash
set -euo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# ROS 2 Humble is installed for Ubuntu's system Python. A globally activated
# Conda environment can make CMake select an incompatible interpreter.
export PATH="/usr/bin:/bin:${PATH}"
unset PYTHONHOME
unset PYTHONPATH

source /opt/ros/humble/setup.bash
cd "${workspace_dir}"
colcon build --symlink-install "$@"

