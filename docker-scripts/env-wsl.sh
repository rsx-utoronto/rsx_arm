#!/usr/bin/env bash
# WSL-specific environment setup for the docker dev container.
# Source this script before running ./docker-scripts/up.sh wsl.

if ! (return 0 2>/dev/null); then
  echo "Please source this script rather than executing it directly:"
  echo "  source ./docker-scripts/env-wsl.sh"
  exit 1
fi

export DISPLAY="${DISPLAY:-:0}"
export WAYLAND_DISPLAY="${WAYLAND_DISPLAY:-wayland-0}"
export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/mnt/wslg/runtime-dir}"

render_device="/dev/dri/renderD128"
if [ -e "$render_device" ]; then
  RENDER_GID="$(stat -c '%g' "$render_device")"
  export RENDER_GID
else
  unset RENDER_GID
  echo "Warning: $render_device not found; skipping RENDER_GID export."
fi

video_device="/dev/video0"
if [ -e "$video_device" ]; then
  VIDEO_GID="$(stat -c '%g' "$video_device")"
  export VIDEO_GID
else
  export VIDEO_GID="0"
  echo "Info: $video_device not found; defaulting VIDEO_GID=0."
fi

echo "WSL environment prepared. You can now run:"
echo "  ./docker-scripts/up.sh wsl"
