#!/usr/bin/env bash
set -euo pipefail
cd "${WS_DIR:-/arm_ros2_ws}"

# Install system deps for the package(s) in this repo
# Using --rosdistro explicit to avoid surprises
sudo apt-get update
rosdep update

# If anything is missing, install; otherwise, skip
if ! rosdep check --from-paths src --ignore-src --rosdistro "${ROS_DISTRO:-humble}" >/dev/null; then
  rosdep install --from-paths src --ignore-src -r -y --rosdistro "${ROS_DISTRO:-humble}"
else
  echo "# rosdep: all dependencies already satisfied."
fi
