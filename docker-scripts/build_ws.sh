#!/usr/bin/env bash
set -Eeuo pipefail

# Workspace root; default to /rover_ws
WS_DIR="${WS_DIR:-/rover_ws}"
: "${ROS_DISTRO:=humble}"

# Always operate from the workspace root
cd "$WS_DIR"

# Source a file with nounset turned off, then restore prior state
source_relaxed() {
  local f="$1"
  [ -f "$f" ] || return 0
  local had_nounset=0
  if [[ -o nounset ]]; then
    had_nounset=1
    set +u
  fi
  . "$f"
  if ((had_nounset)); then set -u; fi
}

# Source ROS 2 env and build
source_relaxed "/opt/ros/${ROS_DISTRO}/setup.sh"
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

# Overlay (for this subshell)
source_relaxed "${WS_DIR}/install/setup.sh"

echo "Build complete at ${WS_DIR}."
echo "Tip: open a new shell (or 'source ${WS_DIR}/install/setup.sh') to overlay your current session."
