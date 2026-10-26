#!/usr/bin/env bash
set -eo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export PATH="/usr/bin:/bin:${PATH}"
unset PYTHONHOME
unset PYTHONPATH

# ROS 环境脚本会读取尚未定义的变量，因此在加载完成后再启用 nounset。
source /opt/ros/humble/setup.bash
set -u

cd "${workspace_dir}"
colcon build --symlink-install "$@"
