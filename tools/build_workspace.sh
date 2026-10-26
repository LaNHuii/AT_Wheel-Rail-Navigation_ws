#!/usr/bin/env bash
set -euo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# ROS 2 Humble 使用 Ubuntu 系统 Python。全局激活的 Conda 环境可能导致
# CMake 选择不兼容的 Python 解释器。
export PATH="/usr/bin:/bin:${PATH}"
unset PYTHONHOME
unset PYTHONPATH

source /opt/ros/humble/setup.bash
cd "${workspace_dir}"
colcon build --symlink-install "$@"
