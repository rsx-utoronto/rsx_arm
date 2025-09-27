#!/usr/bin/env bash
set -euo pipefail

# Workspace root; default to /rover_ws
WS_DIR="${WS_DIR:-/rover_ws}"

# Safety check: don't nuke '/' by accident
if [[ -z "${WS_DIR}" || "${WS_DIR}" == "/" ]]; then
  echo "Refusing to clean: WS_DIR is empty or '/'. Set WS_DIR correctly." >&2
  exit 1
fi

cd "${WS_DIR}"

rm -rf build install log
echo "Cleaned: ${WS_DIR}/build ${WS_DIR}/install ${WS_DIR}/log"
