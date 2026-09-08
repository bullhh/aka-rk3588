#!/bin/bash
# Build script for aka-rk3588
# Cross-compiles on PC using aarch64-linux-gnu toolchain.
# All third-party .so dependencies (libuvc, libusb-1.0, libturbojpeg, librknnrt)
# are resolved at runtime on the board - no host .so files required.
#
# Usage:
#   ./build_rk3588.sh                    # Release
#   ./build_rk3588.sh -b Debug           # Debug
#   ./build_rk3588.sh -b Debug -l DEBUG  # with LOGD output

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

# ── Defaults ──────────────────────────────────────────────────────────────────
BUILD_TYPE=Release
LOG_LEVEL=INFO
GCC_COMPILER=${GCC_COMPILER:-aarch64-linux-gnu}

while getopts ":b:l:" opt; do
  case $opt in
    b) BUILD_TYPE=$OPTARG ;;
    l) LOG_LEVEL=$OPTARG  ;;
    *) echo "Usage: $0 [-b Debug|Release] [-l DEBUG|INFO|WARN|ERROR]"; exit 1 ;;
  esac
done

# ── Auto-detect native vs cross build ────────────────────────────────────────
ARCH=$(uname -m)
if [ "${ARCH}" = "aarch64" ]; then
    echo "  MODE       : native (running on board)"
    NATIVE=ON
    CC=gcc
    CXX=g++
else
    echo "  MODE       : cross-compile"
    NATIVE=OFF
    CC="${GCC_COMPILER}-gcc"
    CXX="${GCC_COMPILER}-g++"
fi

if ! command -v "${CXX}" >/dev/null 2>&1; then
    echo "ERROR: ${CXX} not found."
    if [ "${NATIVE}" = "OFF" ]; then
        echo "Install: sudo apt install gcc-aarch64-linux-gnu g++-aarch64-linux-gnu"
    fi
    exit 1
fi

echo "=== aka-rk3588 build ==="
echo "  BUILD_TYPE : ${BUILD_TYPE}"
echo "  LOG_LEVEL  : ${LOG_LEVEL}"
echo "  CXX        : ${CXX}"
echo ""

BUILD_DIR="${SCRIPT_DIR}/build"
mkdir -p "${BUILD_DIR}"
OUTPUT="${BUILD_DIR}/tennis"
PERCEPTION_OUTPUT="${BUILD_DIR}/tennis-perception"
DUAL_RUNTIME_DIR="${BUILD_DIR}/dual-runtime"

cmake -S "${SCRIPT_DIR}" -B "${BUILD_DIR}" \
    -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" \
    -DCMAKE_C_COMPILER="${CC}" \
    -DCMAKE_CXX_COMPILER="${CXX}" \
    -DLOG_LEVEL="${LOG_LEVEL}" \
    -DTARGET_SOC=rk3588 \
    -DNATIVE_BUILD="${NATIVE}"

EMPTY_ARTIFACTS=$(find "${BUILD_DIR}" -type f \( -name '*.o' -o -name '*.a' -o -name 'tennis' \) -size 0 -print)
if [ -n "${EMPTY_ARTIFACTS}" ]; then
    echo "WARN: removing stale empty build artifacts before rebuild:"
    printf '%s\n' "${EMPTY_ARTIFACTS}"
    printf '%s\n' "${EMPTY_ARTIFACTS}" | xargs rm -f
fi

cmake --build "${BUILD_DIR}" -- -j"$(nproc)"

if [ ! -s "${OUTPUT}" ]; then
    echo "ERROR: build output is missing or empty: ${OUTPUT}" >&2
    exit 1
fi
if [ ! -s "${PERCEPTION_OUTPUT}" ]; then
    echo "ERROR: perception build output is missing or empty: ${PERCEPTION_OUTPUT}" >&2
    exit 1
fi

cmake -E remove_directory "${DUAL_RUNTIME_DIR}"
cmake --install "${BUILD_DIR}" \
    --prefix "${DUAL_RUNTIME_DIR}" \
    --component dual-runtime

if [ ! -x "${DUAL_RUNTIME_DIR}/bin/tennis-perception" ]; then
    echo "ERROR: staged dual-guest runtime is incomplete: ${DUAL_RUNTIME_DIR}" >&2
    exit 1
fi

(
    cd "${DUAL_RUNTIME_DIR}"
    # Calibration and pick configuration are deliberately mutable per robot.
    # Keep them in the package as defaults, but do not make a valid deployed
    # package fail integrity checks after an operator calibrates the robot.
    find . -type f ! -name SHA256SUMS ! -path './config/*' -print0 \
        | sort -z \
        | xargs -0 sha256sum >SHA256SUMS
)

echo ""
echo "=== Build done: ${BUILD_DIR}/tennis ==="
stat -c "    Size: %s bytes" "${OUTPUT}"
echo "=== Dual-guest runtime staged: ${DUAL_RUNTIME_DIR} ==="
find "${DUAL_RUNTIME_DIR}" -maxdepth 2 -type f -printf '    %P\n' | sort
