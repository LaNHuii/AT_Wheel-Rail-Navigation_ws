#!/usr/bin/env bash

# 单独构建rosbag工具，避免Qt和Livox依赖影响导航主工作空间。
set -euo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source_dir="${workspace_dir}/external/rosbag_sensor_trimmer"
artifact_root="${WR_SENSOR_TOOLS_ARTIFACT_ROOT:-/tmp/wr_sensor_tools}"
if [[ ! -f "${source_dir}/package.xml" ]]; then
  printf '未找到rosbag_sensor_trimmer，请先运行 tools/import_optional_tools.sh。\n' >&2
  exit 2
fi
source /opt/ros/humble/setup.bash
if ! ros2 pkg prefix livox_ros_driver2 >/dev/null 2>&1; then
  printf '缺少 livox_ros_driver2；请先按MID-360S驱动说明安装或构建，再编译此可选工具。\n' >&2
  exit 3
fi
mkdir -p "${artifact_root}/build" "${artifact_root}/install" "${artifact_root}/log"
colcon --log-base "${artifact_root}/log" build \
  --base-paths "${source_dir}" \
  --build-base "${artifact_root}/build" \
  --install-base "${artifact_root}/install" \
  --symlink-install
printf '传感器工具构建完成：%s\n' "${artifact_root}/install/setup.bash"
