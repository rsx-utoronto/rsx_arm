#!/usr/bin/env bash
set -euo pipefail
# Bring down JUST THIS PROJECT (compose in current dir) and clean local artifacts.
# Removes containers, anonymous volumes, and images built by this compose file.
# Resolve host home path for compose (matches up.sh)
OS="$(uname -s)"
case "$OS" in
Linux | Darwin) export HOST_HOME="${HOME}" ;;
MINGW* | MSYS* | CYGWIN*) export HOST_HOME="${USERPROFILE}" ;;
*) export HOST_HOME="${HOME}" ;;
esac

docker compose down -v --remove-orphans --rmi local || true
echo "[project] compose down complete."

# Optional: prune dangling stuff left behind
# Only prune when PRUNE=1 ./docker-scripts/down.sh
if [[ "${PRUNE:-0}" == "1" ]]; then
  # keep anything accessed in the last 24h, and keep 10GB of cache
  docker builder prune -f --filter until=24h --keep-storage 10GB || true
  docker image prune -f || true
  docker volume prune -f || true
  docker network prune -f || true
fi
echo "[project] pruned dangling builder cache/images/volumes/networks."
