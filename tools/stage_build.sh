#!/usr/bin/env bash

set -eo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
artifact_root="${WR_NAV_ARTIFACT_ROOT:-/tmp/at_wheel_rail_navigation}"

# ROS 2 Humble's rosidl generator can mis-handle non-ASCII build/install paths.
# Keep source files in the repository while placing generated artifacts in an
# ASCII-only temporary directory. Also prefer the system Python over Conda.
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
