#!/usr/bin/env bash
# Push this working tree to the Orange Pi development directory.
#
# Usage:
#   ./scripts/sync_to_orangepi.sh

set -euo pipefail

REMOTE="${REMOTE:-orangepi@10.3.10.24}"
REMOTE_PROJECT="${REMOTE_PROJECT:-/home/orangepi/robot/aka-rk3588}"

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

echo "Pushing project:"
echo "  ${PROJECT_DIR}/ -> ${REMOTE}:${REMOTE_PROJECT}/"

rsync -az --delete \
    --exclude '.git' \
    --exclude 'build' \
    --exclude 'capture.jpg' \
    --exclude 'result.jpg' \
    --exclude 'imgs' \
    -e "sshpass -p orangepi ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null" \
    "${PROJECT_DIR}/" "${REMOTE}:${REMOTE_PROJECT}/"
