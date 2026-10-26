#!/usr/bin/env bash

# 在纯 ASCII 路径中构建 ROS 2 工作空间，避免 rosidl 生成器对中文路径的兼容性问题。
# 源码仍保留在当前仓库，只将 build、install 和 log 生成到临时目录。

set -eo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
artifact_root="${WR_NAV_ARTIFACT_ROOT:-/tmp/at_wheel_rail_navigation}"

# ROS 2 Humble 的 rosidl 生成器可能无法正确处理非 ASCII 构建/安装路径。
# 因此生成物放在 ASCII 临时路径，并优先使用系统 Python，避免 Conda 环境污染 ROS 2。
export PATH="/usr/bin:/bin:${PATH}"
unset PYTHONHOME PYTHONPATH
source /opt/ros/humble/setup.bash
set -u

mkdir -p "${artifact_root}/build" "${artifact_root}/install" "${artifact_root}/log"

colcon --log-base "${artifact_root}/log" build \
  --base-paths "${workspace_dir}/src" \
  --build-base "${artifact_root}/build" \
  --install-base "${artifact_root}/install" \
  --symlink-install \
  "$@"

printf 'Build complete. Overlay: %s\n' "${artifact_root}/install/setup.bash"
