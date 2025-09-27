#!/usr/bin/env bash
set -euo pipefail
SVC="${1:-rsxarm}"

# Set HOST_HOME just like up.sh does
OS="$(uname -s)"
case "$OS" in
Linux | Darwin) export HOST_HOME="${HOME}" ;;
MINGW* | MSYS* | CYGWIN*) export HOST_HOME="${USERPROFILE}" ;;
*) export HOST_HOME="${HOME}" ;;
esac

# Detect the running container for this service (empty if not running)
CID="$(docker compose ps -q "$SVC" || true)"
if [[ -z "${CID}" ]]; then
  echo "Service '$SVC' is not running. Start it with: ./docker-scripts/up.sh <profile>"
  exit 1
fi

# Prefer zsh if available inside the container
SHELL_BIN="${SHELL_BIN:-/bin/bash}"
if docker compose exec "$SVC" test -x /bin/zsh >/dev/null 2>&1; then
  SHELL_BIN="/bin/zsh"
fi

exec docker compose exec "$SVC" "$SHELL_BIN" -i
