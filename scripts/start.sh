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
  [ -n "${BRIDGE_PID:-}" ] && kill "$BRIDGE_PID" 2>/dev/null || true
  kill -- "-$$" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

export PYTHONPATH="$(rbnx path robonix-api):$PKG:${PYTHONPATH:-}"

# Forward the S10's native drdds chassis topics across the DDS domain boundary.
# The NX stack lives on a private ROS_DOMAIN_ID (RTPS 2.3) while the robot's
# motion controller publishes on LYNX_ROBOT_DOMAIN_ID (RTPS 2.2); sharing one
# domain makes the old discovery pollute the new one and kill the nav2 nodes.
# See chassis_domain_bridge.py. Started BEFORE the primitive so /MOTION_INFO is
# already visible on the private domain when lynx_quadruped subscribes.
python3 "$PKG/scripts/chassis_domain_bridge.py" &
BRIDGE_PID=$!

exec python3 -m lynx_quadruped.main
