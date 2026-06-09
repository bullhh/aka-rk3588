#!/usr/bin/env bash
# Build and run the vision-only RKNN check.
#
# This command intentionally does not initialize motors or the arm. It runs the
# existing `tennis test-yolo` subcommand, captures one UVC frame, runs RKNN
# detection, and writes:
#   - capture.jpg
#   - result.jpg
#
# Usage:
#   ./run_vision_once.sh [model.rknn] [uvc_index]

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
CONDA_RKNN_PREFIX="${CONDA_RKNN_PREFIX:-/home/orangepi/miniforge3/envs/rknn}"

if [[ $# -gt 2 ]]; then
    echo "Usage: $0 [model.rknn] [uvc_index]" >&2
    exit 2
fi

MODEL_PATH="${1:-${SCRIPT_DIR}/models/tennis.rknn}"
UVC_INDEX="${2:-0}"

if [[ ! -f "${MODEL_PATH}" ]]; then
    echo "ERROR: model file not found: ${MODEL_PATH}" >&2
    exit 1
fi

echo "=== Vision-only check ==="
echo "  model     : ${MODEL_PATH}"
echo "  uvc_index : ${UVC_INDEX}"
echo "  action    : build native binary, then run test-yolo once"
echo "  conda env : ${CONDA_RKNN_PREFIX}"
echo ""

if [[ -d "${CONDA_RKNN_PREFIX}" ]]; then
    export PKG_CONFIG_PATH="${CONDA_RKNN_PREFIX}/lib/pkgconfig:${PKG_CONFIG_PATH:-}"
    export LD_LIBRARY_PATH="${CONDA_RKNN_PREFIX}/lib:${LD_LIBRARY_PATH:-}"
fi

"${SCRIPT_DIR}/build_rk3588.sh" -b Release -l INFO

echo ""
echo "=== Running ==="
echo "  ${SCRIPT_DIR}/build/tennis test-yolo ${MODEL_PATH} ${UVC_INDEX}"
echo ""

cd "${SCRIPT_DIR}"
"${SCRIPT_DIR}/build/tennis" test-yolo "${MODEL_PATH}" "${UVC_INDEX}"

echo ""
echo "=== Done ==="
echo "  raw frame : ${SCRIPT_DIR}/capture.jpg"
echo "  result    : ${SCRIPT_DIR}/result.jpg"
