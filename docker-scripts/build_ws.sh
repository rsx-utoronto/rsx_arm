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
  if [[ -o nounset ]]; then had_nounset=1; set +u; fi
  . "$f"
  if ((had_nounset)); then set -u; fi
}

# Source ROS env (shell-agnostic)
source_relaxed "/opt/ros/${ROS_DISTRO}/setup.sh"

# Build with colcon
CMAKE_EXPORT_COMPILE_COMMANDS=ON \
colcon build --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

# Overlay for this shell only (shell-agnostic)
source_relaxed "${WS_DIR}/install/setup.sh"

echo "Build complete at ${WS_DIR}."
echo "Tip: open a new shell (or 'source ${WS_DIR}/install/setup.sh') to overlay your session."

###############################################################################
# Add zsh helpers (aliases) without intermediate vars (nounset-safe)
###############################################################################
ZSHRC="${ZSHRC:-$HOME/.zshrc}"

# Freeze current values into the alias definitions
_ws="$WS_DIR"
_ros="$ROS_DISTRO"

workon_line="alias workon_arm='cd $_ws; source /opt/ros/$_ros/setup.sh; [ -d $_ws/arm_env ] && source $_ws/arm_env/bin/activate || true; source $_ws/install/setup.sh'"
build_line="alias build_arm='workon_arm && colcon build && colcon test --ctest-args tests --packages-skip arm_msgs && colcon test-result --all --verbose'"

mkdir -p "$(dirname "$ZSHRC")"

# Replace or append workon_arm
if grep -qE '^alias[[:space:]]+workon_arm=' "$ZSHRC" 2>/dev/null; then
  sed -i.bak -E "s|^alias[[:space:]]+workon_arm=.*$|$workon_line|g" "$ZSHRC"
else
  printf "\n# ROS 2 helpers for %s (%s)\n%s\n" "$_ws" "$_ros" "$workon_line" >> "$ZSHRC"
fi

# Replace or append build_arm
if grep -qE '^alias[[:space:]]+build_arm=' "$ZSHRC" 2>/dev/null; then
  sed -i.bak -E "s|^alias[[:space:]]+build_arm=.*$|$build_line|g" "$ZSHRC"
else
  printf "%s\n" "$build_line" >> "$ZSHRC"
fi

# Optional formatter alias
if ! grep -qE '^alias[[:space:]]+format_code=' "$ZSHRC" 2>/dev/null; then
  echo "alias format_code='autopep8 --in-place --recursive src/rsx_arm'" >> "$ZSHRC"
fi

echo "Added/updated zsh aliases in $ZSHRC (workon_arm, build_arm, format_code)."
echo "Reload them with:  source \"$ZSHRC\""

###############################################################################
# Run helpers like the original script (inside zsh so aliases exist)
###############################################################################
if command -v zsh >/dev/null 2>&1; then
  zsh -i -c "source \"$ZSHRC\"; workon_arm && build_arm"
else
  echo "⚠ zsh not found; skipping 'workon_arm && build_arm' execution."
fi

# Ensure venv is ignored by colcon (after helpers, as in original)
if [ -d "$WS_DIR/arm_env" ]; then
  : > "$WS_DIR/arm_env/COLCON_IGNORE"
fi

echo
echo "Setup complete. In the future, use 'workon_arm' to enter the workspace and 'build_arm' to build the packages."
echo "Use 'deactivate' to exit the virtual environment when done."
echo

# cd to package dir like the original script
if [ -d "$WS_DIR/src/rsx_arm" ]; then
  cd "$WS_DIR/src/rsx_arm"
fi
