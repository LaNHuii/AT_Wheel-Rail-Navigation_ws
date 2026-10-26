#!/usr/bin/env bash

set -eo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
artifact_root="${WR_NAV_ARTIFACT_ROOT:-/tmp/at_wheel_rail_navigation}"

export PATH="/usr/bin:/bin:${PATH}"
unset PYTHONHOME PYTHONPATH
# 忽略用户目录中与本项目无关的 pytest 插件，例如版本不兼容的 anyio。
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
source /opt/ros/humble/setup.bash
if [[ -f "${artifact_root}/install/setup.bash" ]]; then
  source "${artifact_root}/install/setup.bash"
fi
set -u

colcon --log-base "${artifact_root}/test_log" test \
  --base-paths "${workspace_dir}/src" \
  --build-base "${artifact_root}/build" \
  --install-base "${artifact_root}/install" \
  "$@"

colcon --log-base "${artifact_root}/test_result_log" test-result \
  --test-result-base "${artifact_root}/build" --verbose
