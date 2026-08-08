#!/usr/bin/env bash
set -euo pipefail

GRPC_PORT="${RERUN_GRPC_PORT:-9877}"
WEB_PORT="${RERUN_WEB_PORT:-9091}"
CONFIG="${LIVO_SLAM_CONFIG:-/opt/livo-slam/config/livo_slam.yaml}"
BAG="${LIVO_SLAM_BAG:-}"

rerun --serve-web --port "${GRPC_PORT}" --web-viewer-port "${WEB_PORT}" \
  >/tmp/rerun.log 2>&1 &
RERUN_PID=$!
trap 'kill -TERM "${RERUN_PID}" 2>/dev/null || true' EXIT

for _ in $(seq 1 40); do
  if (echo >"/dev/tcp/127.0.0.1/${WEB_PORT}") 2>/dev/null; then
    break
  fi
  sleep 0.25
done

URL="http://127.0.0.1:${WEB_PORT}?url=rerun%2Bhttp%3A%2F%2Flocalhost%3A${GRPC_PORT}%2Fproxy"
echo "================================================================"
echo " Rerun viewer: ${URL}"
echo "================================================================"

NODE_ARGS=(--ros-args --params-file "${CONFIG}"
           -p "viz.grpc_url:=rerun+http://127.0.0.1:${GRPC_PORT}/proxy" "$@")

if [[ -z "${BAG}" ]]; then
  exec livo_slam "${NODE_ARGS[@]}"
fi

livo_slam "${NODE_ARGS[@]}" &
NODE_PID=$!
trap 'kill -TERM "${NODE_PID}" "${RERUN_PID}" 2>/dev/null || true' EXIT

for _ in $(seq 1 40); do
  if ros2 node list 2>/dev/null | grep -q "/livo_slam"; then
    break
  fi
  sleep 0.25
done

ros2 bag play "${BAG}" --delay 2

echo "playback finished; viewer still serving ${URL}"
wait "${RERUN_PID}"
