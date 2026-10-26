#!/usr/bin/env bash

# 从长时间真车记录中截取可复现片段，并保留本项目关键传感器与状态话题。
set -euo pipefail

if [[ $# -ne 4 ]]; then
  printf '用法：%s <输入bag> <输出bag> <起始秒> <结束秒>\n' "$0" >&2
  exit 2
fi
artifact_root="${WR_SENSOR_TOOLS_ARTIFACT_ROOT:-/tmp/wr_sensor_tools}"
source /opt/ros/humble/setup.bash
if [[ ! -f "${artifact_root}/install/setup.bash" ]]; then
  printf '传感器工具尚未构建，请先运行 tools/build_sensor_tools.sh。\n' >&2
  exit 2
fi
source "${artifact_root}/install/setup.bash"
ros2 run rosbag_sensor_trimmer rosbag_sensor_trimmer_cli \
  --input "$1" --output "$2" --start "$3" --end "$4" \
  --topics \
    /livox/lidar /imu/data_raw /imu/status /chassis_state \
    /wheel/odometry /odometry/filtered /tf /tf_static \
    /local_safety_state /local_traversability /local_obstacle_grid \
    /rail_entry_estimate /rail_engagement_state /rail_clamp_feedback \
    /system_health /tracked_person /person_follow_status \
    /cmd_vel_selected /cmd_vel_chassis \
  --compression zstd --compression-mode file --verify
