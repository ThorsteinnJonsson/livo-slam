#!/usr/bin/env bash
# Waits for the livo_slam node to appear, then plays a bag. Launched detached by
# the "play bag" VS Code task; stopped by killing its process group.
set -euo pipefail

BAG="${1:-/workspace/datasets/ntu_viral/eee_03_mcap}"

echo "$$" >/tmp/livo_bag.pid
echo "waiting for /livo_slam"

for _ in $(seq 1 120); do
  if ros2 node list 2>/dev/null | grep -q "/livo_slam"; then
    break
  fi
  sleep 0.25
done

echo "playing ${BAG}"
exec ros2 bag play "${BAG}" --delay 1
