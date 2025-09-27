#!/usr/bin/env bash
set -euo pipefail
PROFILE="${1:-base}" # base|hyprland|x11|mac|windows

# Resolve host home path for compose
OS="$(uname -s)"
case "$OS" in
Linux | Darwin) export HOST_HOME="${HOME}" ;;
MINGW* | MSYS* | CYGWIN*) export HOST_HOME="${USERPROFILE}" ;; # Git Bash/Windows
*) export HOST_HOME="${HOME}" ;;
esac

export HOST_UID="${UID:-$(id -u)}"
export HOST_GID="$(id -g)"

case "$PROFILE" in
hyprland) svc="rsxarm-wayland" ;;
x11) svc="rsxarm-x11" ;;
mac) svc="rsxarm-mac" ;;
windows) svc="rsxarm-win" ;;
*) svc="rsxarm" ;;
esac

docker compose up -d "$svc"
echo "Container up with profile: $PROFILE  (HOST_HOME=$HOST_HOME)"
