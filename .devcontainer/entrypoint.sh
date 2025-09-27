#!/usr/bin/env bash
set -e

# Source ROS
if [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
fi

# Sync dotfiles from host (if mount is present)
if [ "${SYNC_DOTFILES_ON_START:-0}" = "1" ] && [ -d "/host_home" ]; then
  /usr/local/bin/sync_dotfiles.sh || true
fi

# Overlay workspace if present
WS_DIR="${WS_DIR:-/arm_ros2_ws}"
if [ -f "${WS_DIR}/install/setup.bash" ]; then
  source "${WS_DIR}/install/setup.bash"
fi

exec "$@"
