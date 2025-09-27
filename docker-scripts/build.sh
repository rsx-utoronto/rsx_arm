#!/usr/bin/env bash
set -euo pipefail

SHELL_FLAVOR="${1:-zsh}"   # zsh|bash
EDITOR_FLAVOR="${2:-nvim}" # nvim|vscode

# Set HOST_HOME just like up.sh does
OS="$(uname -s)"
case "$OS" in
Linux | Darwin) export HOST_HOME="${HOME}" ;;
MINGW* | MSYS* | CYGWIN*) export HOST_HOME="${USERPROFILE}" ;;
*) export HOST_HOME="${HOME}" ;;
esac

export SHELL_FLAVOR EDITOR_FLAVOR
export HOST_UID="${UID:-$(id -u)}"
export HOST_GID="$(id -g)"

docker compose build rsxrover
echo "Built image with SHELL=${SHELL_FLAVOR}, EDITOR=${EDITOR_FLAVOR}, HOST_HOME=${HOST_HOME}"
