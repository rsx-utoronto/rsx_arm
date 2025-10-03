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

# --- ensure user-writable runtime + config dirs for GUI apps (VS Code, etc.) ---
export XDG_RUNTIME_DIR="/run/user/$(id -u)"
sudo mkdir -p "$XDG_RUNTIME_DIR" && sudo chown "$(id -u)":"$(id -g)" "$XDG_RUNTIME_DIR" && chmod 700 "$XDG_RUNTIME_DIR"

# Create user config dirs if missing
mkdir -p "$HOME/.config" "$HOME/.vscode"

# If created by root in earlier runs, fix ownership now
if [ "$(stat -c %U "$HOME/.config")" != "$(id -un)" ]; then
  sudo chown -R "$(id -u)":"$(id -g)" "$HOME/.config"
fi
if [ "$(stat -c %U "$HOME/.vscode")" != "$(id -un)" ]; then
  sudo chown -R "$(id -u)":"$(id -g)" "$HOME/.vscode"
fi

# Reasonable perms
chmod -R u+rwX,go-rwx "$HOME/.config" "$HOME/.vscode" 2>/dev/null || true

exec "$@"
