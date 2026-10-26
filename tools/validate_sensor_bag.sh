#!/usr/bin/env bash

# 使用固定版本的rosbag工具验证已有数据包，不修改原始bag。
set -euo pipefail

if [[ $# -lt 1 ]]; then
  printf '用法：%s <rosbag2目录> [报告JSON路径]\n' "$0" >&2
  exit 2
fi
artifact_root="${WR_SENSOR_TOOLS_ARTIFACT_ROOT:-/tmp/wr_sensor_tools}"
source /opt/ros/humble/setup.bash
if [[ ! -f "${artifact_root}/install/setup.bash" ]]; then
  printf '传感器工具尚未构建，请先运行 tools/build_sensor_tools.sh。\n' >&2
  exit 2
fi
source "${artifact_root}/install/setup.bash"
command=(ros2 run rosbag_sensor_trimmer rosbag_sensor_trimmer_cli --input "$1" --verify-only)
if [[ $# -ge 2 ]]; then
  command+=(--report "$2")
fi
"${command[@]}"
