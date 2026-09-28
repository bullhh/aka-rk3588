#!/bin/bash
# Build script for aka-rk3588
# Cross-compiles on PC using aarch64-linux-gnu toolchain.
#
# The cross build links the real AArch64 runtime libraries (libuvc, libusb-1.0,
# libturbojpeg, plus libudev.so.1 which libusb-1.0.so needs transitively) taken
# from the target rootfs. The former empty-stub scheme is forbidden: it produced
# a tennis binary whose uvc_* calls had no dynamic relocation and crashed
# (SIGILL) in UvcCapture::open. On a non-aarch64 host a real target library
# directory is therefore mandatory; the board-native (aarch64) build resolves
# them from the installed system libraries instead.
#
# Usage:
#   ./build_rk3588.sh                                        # native (aarch64 board)
#   ./build_rk3588.sh -b Debug                               # Debug
#   ./build_rk3588.sh -b Debug -l DEBUG                      # with LOGD output
#   ./build_rk3588.sh -L /path/to/jammy-aarch64-libs         # cross build
#   AKA_RK3588_CROSS_LIB_DIR=/path/to/jammy-aarch64-libs ./build_rk3588.sh
#
# The cross library directory must contain the real AArch64 files:
#   libuvc.so  libusb-1.0.so  libturbojpeg.so  libudev.so.1

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
CROSS_RUNTIME_LIBS="libuvc.so libusb-1.0.so libturbojpeg.so libudev.so.1"

# ── Defaults ──────────────────────────────────────────────────────────────────
BUILD_TYPE=Release
LOG_LEVEL=INFO
GCC_COMPILER=${GCC_COMPILER:-aarch64-linux-gnu}
# Non-aarch64 cross builds must point at the real target libraries, either with
# -L <dir> or with AKA_RK3588_CROSS_LIB_DIR.
CROSS_LIB_DIR=${AKA_RK3588_CROSS_LIB_DIR:-}

while getopts ":b:l:L:" opt; do
  case $opt in
    b) BUILD_TYPE=$OPTARG ;;
    l) LOG_LEVEL=$OPTARG  ;;
    L) CROSS_LIB_DIR=$OPTARG ;;
    *) echo "Usage: $0 [-b Debug|Release] [-l DEBUG|INFO|WARN|ERROR] [-L cross-lib-dir]"; exit 1 ;;
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

# ── Cross build safety gate ───────────────────────────────────────────────────
# The default AArch64 build links the real target libraries from the extracted
# Jammy rootfs. Refuse to configure without them rather than emit a broken
# binary: this check runs before any cmake invocation.
if [ "${NATIVE}" = "OFF" ]; then
    if [ -z "${CROSS_LIB_DIR}" ]; then
        echo "ERROR: cross build requires the real AArch64 target libraries." >&2
        echo "  Pass -L <dir> or set AKA_RK3588_CROSS_LIB_DIR=<dir>." >&2
        echo "  The directory must contain: ${CROSS_RUNTIME_LIBS}" >&2
        echo "  Empty stub libraries are forbidden (they produced SIGILL binaries)." >&2
        echo "  On an aarch64 board the native build needs no such directory." >&2
        exit 1
    fi
    if [ ! -d "${CROSS_LIB_DIR}" ]; then
        echo "ERROR: cross library directory does not exist: ${CROSS_LIB_DIR}" >&2
        exit 1
    fi
    MISSING_LIBS=""
    for lib in ${CROSS_RUNTIME_LIBS}; do
        if [ ! -f "${CROSS_LIB_DIR}/${lib}" ]; then
            MISSING_LIBS="${MISSING_LIBS} ${CROSS_LIB_DIR}/${lib}"
        fi
    done
    if [ -n "${MISSING_LIBS}" ]; then
        echo "ERROR: cross library directory is missing required AArch64 libraries:" >&2
        for lib in ${MISSING_LIBS}; do
            echo "  ${lib}" >&2
        done
        echo "  Expected: ${CROSS_RUNTIME_LIBS}" >&2
        exit 1
    fi
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
if [ "${NATIVE}" = "OFF" ]; then
    echo "  CROSS LIBS : ${CROSS_LIB_DIR}"
fi
echo ""

BUILD_DIR="${SCRIPT_DIR}/build"
mkdir -p "${BUILD_DIR}"
OUTPUT="${BUILD_DIR}/tennis"

CROSS_LIB_CMAKE_FLAG=""
if [ "${NATIVE}" = "OFF" ]; then
    CROSS_LIB_CMAKE_FLAG="-DTARGET_RUNTIME_LIB_DIR=${CROSS_LIB_DIR}"
fi

cmake -S "${SCRIPT_DIR}" -B "${BUILD_DIR}" \
    -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" \
    -DCMAKE_C_COMPILER="${CC}" \
    -DCMAKE_CXX_COMPILER="${CXX}" \
    -DLOG_LEVEL="${LOG_LEVEL}" \
    -DTARGET_SOC=rk3588 \
    -DNATIVE_BUILD="${NATIVE}" \
    ${CROSS_LIB_CMAKE_FLAG}

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

echo ""
echo "=== Build done: ${BUILD_DIR}/tennis ==="
stat -c "    Size: %s bytes" "${OUTPUT}"
echo "    Deploy: scp ${BUILD_DIR}/tennis root@<board_ip>:/root/"
