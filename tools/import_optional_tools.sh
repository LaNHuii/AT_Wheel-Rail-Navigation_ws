#!/usr/bin/env bash

# 按固定提交导入可选数据工具。外部源码位于 external/，不参加主工作空间构建。
set -euo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if ! command -v vcs >/dev/null 2>&1; then
  printf '缺少 vcs，请先安装 python3-vcstool。\n' >&2
  exit 2
fi
mkdir -p "${workspace_dir}/external"
vcs import "${workspace_dir}/external" < "${workspace_dir}/tools/optional_tools.repos"
vcs status "${workspace_dir}/external"
