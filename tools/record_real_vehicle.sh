#!/usr/bin/env bash

# 记录轨道识别、定位和电控联调所需的最小真车数据集。
set -euo pipefail

output="${1:-wr_sensor_$(date +%Y%m%d_%H%M%S)}"
storage="${WR_ROSBAG_STORAGE:-sqlite3}"
source /opt/ros/humble/setup.bash
ros2 bag record \
  --output "${output}" \
  --storage "${storage}" \
  --compression-mode file \
  --compression-format zstd \
  /livox/lidar \
  /imu/data_raw \
  /imu/status \
  /chassis_state \
  /wheel/odometry \
  /odometry/filtered \
  /tf \
  /tf_static \
  /local_safety_state \
  /local_traversability \
  /local_obstacle_grid \
  /rail_entry_estimate \
  /rail_engagement_state \
  /rail_clamp_feedback \
  /system_health \
  /tracked_person \
  /person_follow_status \
  /cmd_vel_selected \
  /cmd_vel_chassis
