#!/usr/bin/env bash
set -e

WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

docker build -t f1tenth_gym_ros:dev -f "${WS_ROOT}/src/f1tenth_gym_ros/Dockerfile" "${WS_ROOT}"
