#!/bin/sh

set -eu

SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" >/dev/null 2>&1 && pwd -P)
PERCEPTION_BIN=${PERCEPTION_BIN:-${SCRIPT_DIR}/build-ivc-sdk-native/tennis-perception}
MODEL_PATH=${MODEL_PATH:-${SCRIPT_DIR}/models/tennis.rknn}
CAMERA_INDEX=${CAMERA_INDEX:-0}
RKNN_CORE_MASK=${RKNN_CORE_MASK:-0}
REPORT_EVERY=${REPORT_EVERY:-1}
STATUS_EVERY=${STATUS_EVERY:-60}
PIPELINE_HEARTBEAT=${PIPELINE_HEARTBEAT:-0}
IVC_KEY=${IVC_KEY:-0x49564301}
IVC_SIZE=${IVC_SIZE:-65536}
AXIVC_DEVICE=${AXIVC_DEVICE:-/dev/axivc}
AXVISOR_KO=${AXVISOR_KO:-${SCRIPT_DIR}/axvisor.ko}

# StarryOS provides /dev/axivc from the kernel.  Linux obtains the same
# device from AxVisor's kernel module.  Checking the device first keeps this
# entry point usable in both guests without trying to load a module in
# StarryOS.
if [ ! -c "${AXIVC_DEVICE}" ]; then
    if [ "$(id -u)" -ne 0 ]; then
        echo "ERROR: ${AXIVC_DEVICE} is missing; loading ${AXVISOR_KO} requires root privileges" >&2
        exit 1
    fi
    if [ ! -f "${AXVISOR_KO}" ]; then
        echo "ERROR: ${AXIVC_DEVICE} is missing and AxVisor Linux driver is unavailable: ${AXVISOR_KO}" >&2
        exit 1
    fi
    echo "AXIVC_LOAD module=${AXVISOR_KO}"
    if ! insmod "${AXVISOR_KO}"; then
        echo "ERROR: failed to load AxVisor Linux driver: ${AXVISOR_KO}" >&2
        exit 1
    fi
fi

if [ ! -c "${AXIVC_DEVICE}" ]; then
    echo "ERROR: AXIVC manager device was not created: ${AXIVC_DEVICE}" >&2
    exit 1
fi

echo "AXIVC_READY device=${AXIVC_DEVICE}"

if [ ! -x "${PERCEPTION_BIN}" ]; then
    echo "ERROR: perception program is missing or not executable: ${PERCEPTION_BIN}" >&2
    exit 1
fi
if [ ! -f "${MODEL_PATH}" ]; then
    echo "ERROR: RKNN model is missing: ${MODEL_PATH}" >&2
    exit 1
fi

exec env RKNN_CORE_MASK="${RKNN_CORE_MASK}" \
    "${PERCEPTION_BIN}" "${MODEL_PATH}" "${CAMERA_INDEX}" \
    --transport ivc \
    --ivc-key "${IVC_KEY}" \
    --ivc-size "${IVC_SIZE}" \
    --report-every "${REPORT_EVERY}" \
    --status-every "${STATUS_EVERY}" \
    --pipeline-heartbeat "${PIPELINE_HEARTBEAT}" \
    "$@"
