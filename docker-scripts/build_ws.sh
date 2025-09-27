#!/usr/bin/env bash
set -Eeuo pipefail

# Workspace root; default to /arm_ros2_ws
WS_DIR="${WS_DIR:-/arm_ros2_ws}"
: "${ROS_DISTRO:=humble}"

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

# Prefer zsh setup if present, otherwise generic .sh
if [ -f "/opt/ros/${ROS_DISTRO}/setup.zsh" ]; then
  source_relaxed "/opt/ros/${ROS_DISTRO}/setup.zsh"
else
  source_relaxed "/opt/ros/${ROS_DISTRO}/setup.sh"
fi

# Build with colcon
colcon build --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

# Overlay for this shell only
if [ -f "${WS_DIR}/install/setup.zsh" ]; then
  source_relaxed "${WS_DIR}/install/setup.zsh"
else
  source_relaxed "${WS_DIR}/install/setup.sh"
fi

echo "Build complete at ${WS_DIR}."
echo "Tip: open a new shell (or 'source ${WS_DIR}/install/setup.zsh') to overlay your session."

###############################################################################
# Add zsh helpers: workon_arm & build_arm (idempotent)
###############################################################################
ZSHRC="${ZSHRC:-$HOME/.zshrc}"

workon_arm_body=$'cd '"$WS_DIR"$'; ' \
  $'[ -f /opt/ros/'"$ROS_DISTRO"$'/setup.zsh ] && source /opt/ros/'"$ROS_DISTRO"$'/setup.zsh || source /opt/ros/'"$ROS_DISTRO"$'/setup.sh; ' \
  $'[ -d '"$WS_DIR"$'/arm_env ] && source '"$WS_DIR"$'/arm_env/bin/activate || true; ' \
  $'[ -f '"$WS_DIR"$'/install/setup.zsh ] && source '"$WS_DIR"$'/install/setup.zsh || source '"$WS_DIR"$'/install/setup.sh'

build_arm_body=$'workon_arm && colcon build && colcon test --ctest-args tests --packages-skip arm_msgs && colcon test-result --all --verbose'

add_or_replace_alias() {
  local name="$1" body="$2"
  if grep -qE "^alias ${name}=" "$ZSHRC" 2>/dev/null; then
    sed -i.bak -E "s|^alias ${name}=.*$|alias ${name}='$body'|g" "$ZSHRC"
  else
    printf "\n# ROS 2 helpers for %s (%s)\n" "$WS_DIR" "$ROS_DISTRO" >>"$ZSHRC"
    printf "alias %s='%s'\n" "$name" "$body" >>"$ZSHRC"
  fi
}

mkdir -p "$(dirname "$ZSHRC")"
add_or_replace_alias "workon_arm" "$workon_arm_body"
add_or_replace_alias "build_arm" "$build_arm_body"
if ! grep -qE "^alias format_code=" "$ZSHRC" 2>/dev/null; then
  echo "alias format_code='autopep8 --in-place --recursive src/rsx_arm'" >>"$ZSHRC"
fi

###############################################################################
# NEW: Run the two helper commands (in zsh), then touch COLCON_IGNORE, print msgs
###############################################################################
# Ensure zsh exists
if command -v zsh >/dev/null 2>&1; then
  # Run inside an interactive zsh so ~/.zshrc (and aliases) are loaded.
  # If build/tests fail, the script stops due to set -e.
  zsh -i -c "source \"$ZSHRC\"; workon_arm && build_arm"
else
  echo "⚠ zsh not found; skipping 'workon_arm && build_arm' execution."
fi

# Make sure colcon doesn't try to build the virtual environment
# (touch after helpers to mirror original behavior)
if [ -d "$WS_DIR/arm_env" ]; then
  touch "$WS_DIR/arm_env/COLCON_IGNORE"
fi

echo
echo "Setup complete. In the future, use 'workon_arm' to enter the workspace and 'build_arm' to build the packages."
echo "Use 'deactivate' to exit the virtual environment when done."
echo

# Finally cd to the package dir like the original script
if [ -d "$WS_DIR/src/rsx_arm" ]; then
  cd "$WS_DIR/src/rsx_arm"
fi
