#!/usr/bin/env bash
set -e

WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE_TAG="f1tenth_gym_ros:dev"

# X11 권한 허용
if command -v xhost >/dev/null 2>&1; then
  xhost +SI:localuser:$(whoami) >/dev/null 2>&1 || true
fi

ARGS=(
  -it --rm
  --net=host
  --user "$(id -u)":"$(id -g)"
  -e "DISPLAY=${DISPLAY:-:0}"
  -e QT_X11_NO_MITSHM=1
  -v /tmp/.X11-unix:/tmp/.X11-unix
  -v "${WS_ROOT}/src":/sim_ws/src
  -v /etc/passwd:/etc/passwd:ro
  -v /etc/group:/etc/group:ro
)

# GPU 자동 선택
if command -v nvidia-smi >/dev/null 2>&1; then
  ARGS+=(--gpus all -e NVIDIA_DRIVER_CAPABILITIES=all)
else
  [ -e /dev/dri ] && ARGS+=(--device=/dev/dri)
fi

docker run "${ARGS[@]}" "${IMAGE_TAG}" bash
