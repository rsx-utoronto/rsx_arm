#!/usr/bin/env bash
set -euo pipefail
cd "${WS_DIR:-/rover_ws}"

# Install system deps for the package(s) in this repo
# Using --rosdistro explicit to avoid surprises
sudo apt-get update
rosdep update
rosdep install --from-paths . --ignore-src -r -y --rosdistro "$ROS_DISTRO"
