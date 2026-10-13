#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# lynx_quadruped runtime — start the ROS-aware chassis primitive.
set -euo pipefail

PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"

# drdds is the S10's custom ROS 2 message package, prebuilt on the Jetson.
# Override with LYNX_DRDDS_INSTALL if it lives elsewhere.
DRDDS_INSTALL="${LYNX_DRDDS_INSTALL:-/home/nvidia/drdds_build_ws/install}"

# ROS setup scripts may reference unset variables; disable nounset temporarily.
set +u
if [ -f "/opt/ros/humble/setup.bash" ]; then
  source /opt/ros/humble/setup.bash
fi
if [ -f "$DRDDS_INSTALL/setup.bash" ]; then
  source "$DRDDS_INSTALL/setup.bash"
fi
set -u

cleanup() {
  kill -- "-$$" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

export PYTHONPATH="$(rbnx path robonix-api):$PKG:${PYTHONPATH:-}"

exec python3 -m lynx_quadruped.main
