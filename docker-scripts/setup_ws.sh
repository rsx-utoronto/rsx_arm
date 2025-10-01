#!/usr/bin/env bash
set -Eeuo pipefail

# Workspace root; default to /arm_ros2_ws
WS_DIR="${WS_DIR:-/arm_ros2_ws}"
: "${ROS_DISTRO:=humble}"
: "${BUILD_TYPE:=RelWithDebInfo}"

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

echo "==> Sourcing ROS: /opt/ros/${ROS_DISTRO}/setup.sh"
source_relaxed "/opt/ros/${ROS_DISTRO}/setup.sh"

# ----------------- Optional system deps via rosdep -----------------
# (Uncomment if you want this script to ensure deps inside the container)
# rosdep update
# rosdep install --from-paths src --ignore-src -r -y

# ----------------- Build -----------------
echo "==> Building workspace (colcon)"
colcon build --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
  "$@"

# Overlay for this shell
echo "==> Sourcing overlay: ${WS_DIR}/install/setup.sh"
source_relaxed "${WS_DIR}/install/setup.sh"

# Install the requirements
echo "==> Install python packages: ${WS_DIR}/src/rsx_arm/requirements.txt"
python3 -m pip install --upgrade pip setuptools wheel
python3 -m pip install --no-cache-dir -r "${WS_DIR}/src/rsx_arm/requirements.txt"

echo "Build complete at ${WS_DIR}."
echo "Tip: open a new shell (or 'source ${WS_DIR}/install/setup.sh') to overlay your current session."
