#!/usr/bin/env bash
# Pull the current project copy and common RKNN model from the Orange Pi.
#
# Usage:
#   ./scripts/sync_from_orangepi.sh

set -euo pipefail

REMOTE="${REMOTE:-orangepi@10.3.10.24}"
REMOTE_PROJECT="${REMOTE_PROJECT:-/home/orangepi/robot/aka-rk3588}"
REMOTE_MODEL="${REMOTE_MODEL:-/home/orangepi/Code/Desktop-Wanderer/src/yolov/models/tennis.rknn}"

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

mkdir -p "${PROJECT_DIR}/models"

echo "Pulling model:"
echo "  ${REMOTE}:${REMOTE_MODEL}"
sshpass -p orangepi scp -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null \
    "${REMOTE}:${REMOTE_MODEL}" "${PROJECT_DIR}/models/tennis.rknn"

echo "Pulling remote project copy:"
echo "  ${REMOTE}:${REMOTE_PROJECT}/"
rsync -az --delete \
    --exclude '.git' \
    --exclude 'build' \
    --exclude 'capture.jpg' \
    --exclude 'result.jpg' \
    --exclude 'imgs' \
    -e "sshpass -p orangepi ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null" \
    "${REMOTE}:${REMOTE_PROJECT}/" "${PROJECT_DIR}/"
