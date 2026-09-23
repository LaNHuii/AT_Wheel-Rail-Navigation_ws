#!/usr/bin/env bash
set -eo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export PATH="/usr/bin:/bin:${PATH}"
unset PYTHONHOME
unset PYTHONPATH

source /opt/ros/humble/setup.bash
source "${workspace_dir}/install/setup.bash"
set -u

ros2 interface show wr_interfaces/msg/SwerveState >/dev/null
ros2 interface show wr_interfaces/msg/RailEngagementState >/dev/null
ros2 interface show wr_interfaces/msg/RailEntryEstimate >/dev/null
ros2 interface show wr_interfaces/srv/SetRailMode >/dev/null

echo "wr_interfaces: all generated interfaces are discoverable"

