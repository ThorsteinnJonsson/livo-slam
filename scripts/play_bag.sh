#!/usr/bin/env bash
# Waits for the livo_slam node to appear, then plays a bag. Defaults to the
# datasets/active symlink set by select_dataset.sh. Launched detached by the
# "play bag" VS Code task; stopped by killing its process group.
set -euo pipefail

WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BAG="${1:-${WORKSPACE}/datasets/active}"

if [[ ! -d "${BAG}" ]]; then
  echo "bag ${BAG} does not exist; run select_dataset.sh first" >&2
  exit 1
fi

echo "$$" >/tmp/livo_bag.pid
echo "waiting for /livo_slam"

for _ in $(seq 1 120); do
  if ros2 node list 2>/dev/null | grep -q "/livo_slam"; then
    break
  fi
  sleep 0.25
done

echo "playing $(readlink -f "${BAG}")"
exec ros2 bag play "${BAG}" --delay 1
