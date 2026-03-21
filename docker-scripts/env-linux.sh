#!/usr/bin/env bash
# Linux desktop environment setup helper for the docker dev container.
# Source this script before running ./docker-scripts/up.sh <profile>.

if ! (return 0 2>/dev/null); then
  echo "Please source this script rather than executing it directly:"
  echo "  source ./docker-scripts/env-linux.sh [hyprland|x11]"
  exit 1
fi

profile="${1:-hyprland}"

# Resolve the runtime directory if it exists.
default_runtime="/run/user/$(id -u)"
if [ -z "${XDG_RUNTIME_DIR:-}" ] && [ -d "$default_runtime" ]; then
  export XDG_RUNTIME_DIR="$default_runtime"
fi

case "$profile" in
hyprland | wayland)
  export WAYLAND_DISPLAY="${WAYLAND_DISPLAY:-wayland-1}"
  ;;
x11)
  export DISPLAY="${DISPLAY:-:0}"
  ;;
*)
  echo "Info: Unknown profile '$profile'; assuming hyprland defaults."
  export WAYLAND_DISPLAY="${WAYLAND_DISPLAY:-wayland-1}"
  ;;
esac

# Resolve GPU render group for Wayland/X11 acceleration.
render_device="/dev/dri/renderD128"
if [ -e "$render_device" ]; then
  RENDER_GID="$(stat -c '%g' "$render_device")"
  export RENDER_GID
else
  unset RENDER_GID
  echo "Warning: $render_device not found; skipping RENDER_GID export."
fi

# Resolve camera/video group if present; fall back to 0 (root) if absent.
video_device="/dev/video0"
if [ -e "$video_device" ]; then
  VIDEO_GID="$(stat -c '%g' "$video_device")"
  export VIDEO_GID
else
  export VIDEO_GID="0"
  echo "Info: $video_device not found; defaulting VIDEO_GID=0."
fi

# Allow local X11 clients if xhost is available (helps RViz/Gazebo fallback).
if command -v xhost >/dev/null 2>&1; then
  if [ -n "${DISPLAY:-}" ]; then
    xhost +local: >/dev/null
  else
    echo "Info: DISPLAY not set; skipping xhost +local: call."
  fi
else
  echo "Info: xhost not available; skipping X11 access grant."
fi

echo "Linux environment prepared for profile '$profile'. You can now run:"
echo "  ./docker-scripts/build.sh zsh nvim"
