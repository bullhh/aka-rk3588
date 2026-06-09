#!/usr/bin/env bash
# Build and run the C++ closed loop on the LeKiwi/STS3215 platform.
#
# Usage:
#   ./run_lekiwi_loop.sh [model.rknn] [feetech_dev] [uvc_index]

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
CONDA_RKNN_PREFIX="${CONDA_RKNN_PREFIX:-/home/orangepi/miniforge3/envs/rknn}"

MODEL_PATH="${1:-${SCRIPT_DIR}/models/tennis.rknn}"
FEETECH_DEV="${2:-/dev/ttyACM0}"
UVC_INDEX="${3:-0}"

if [[ ! -f "${MODEL_PATH}" ]]; then
    echo "ERROR: model file not found: ${MODEL_PATH}" >&2
    exit 1
fi

if [[ -d "${CONDA_RKNN_PREFIX}" ]]; then
    export PKG_CONFIG_PATH="${CONDA_RKNN_PREFIX}/lib/pkgconfig:${PKG_CONFIG_PATH:-}"
    export LD_LIBRARY_PATH="${CONDA_RKNN_PREFIX}/lib:${LD_LIBRARY_PATH:-}"
fi

echo "=== LeKiwi closed loop ==="
echo "  model       : ${MODEL_PATH}"
echo "  feetech_dev : ${FEETECH_DEV}"
echo "  uvc_index   : ${UVC_INDEX}"
echo "  conda env   : ${CONDA_RKNN_PREFIX}"
echo ""

"${SCRIPT_DIR}/build_rk3588.sh" -b Release -l INFO

echo ""
echo "=== Running ==="
echo "  ${SCRIPT_DIR}/build/tennis ${MODEL_PATH} ${FEETECH_DEV} ${UVC_INDEX} ${FEETECH_DEV} lekiwi"
echo ""

cd "${SCRIPT_DIR}"
"${SCRIPT_DIR}/build/tennis" "${MODEL_PATH}" "${FEETECH_DEV}" "${UVC_INDEX}" "${FEETECH_DEV}" lekiwi
