#!/usr/bin/env bash
set -eo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export PATH="/usr/bin:/bin:${PATH}"
unset PYTHONHOME
unset PYTHONPATH

# ROS setup scripts read variables that may not exist yet, so nounset is enabled
# only after sourcing them.
source /opt/ros/humble/setup.bash
set -u

cd "${workspace_dir}"
colcon build --symlink-install "$@"

