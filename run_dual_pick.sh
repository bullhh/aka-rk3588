#!/bin/sh

set -eu

if [ "$#" -ne 0 ]; then
    echo "ERROR: run_dual_pick.sh does not accept arguments" >&2
    exit 2
fi

SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" >/dev/null 2>&1 && pwd -P)
PERCEPTION_BIN="${SCRIPT_DIR}/bin/tennis-perception"
MODEL_PATH="${SCRIPT_DIR}/models/tennis.rknn"
CALIBRATION_PATH="${SCRIPT_DIR}/config/lekiwi_calibration.json"
PICK_CONFIG_PATH="${SCRIPT_DIR}/config/lekiwi_pick_config.txt"
CAMERA_INDEX=0
IVC_KEY=0x49564301
IVC_SIZE=65536
AXIVC_DEVICE=/dev/axivc
AXVISOR_KO="${SCRIPT_DIR}/axvisor.ko"
AXIVC_MODULE_LOADED=0

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
    if command -v modinfo >/dev/null 2>&1; then
        kernel_release=$(uname -r)
        module_vermagic=$(modinfo -F vermagic "${AXVISOR_KO}" 2>/dev/null || true)
        case "${module_vermagic}" in
            "${kernel_release}"*) ;;
            *)
                echo "ERROR: AxVisor module vermagic does not match the running kernel" >&2
                echo "  kernel  : ${kernel_release}" >&2
                echo "  vermagic: ${module_vermagic:-unavailable}" >&2
                exit 1
                ;;
        esac
    fi
    echo "AXIVC_LOAD module=${AXVISOR_KO}"
    if ! insmod "${AXVISOR_KO}"; then
        echo "ERROR: failed to load AxVisor Linux driver: ${AXVISOR_KO}" >&2
        exit 1
    fi
    AXIVC_MODULE_LOADED=1
    elapsed=0
    while [ ! -c "${AXIVC_DEVICE}" ] && [ "${elapsed}" -lt 2 ]; do
        sleep 1
        elapsed=$((elapsed + 1))
    done
fi

if [ ! -c "${AXIVC_DEVICE}" ]; then
    echo "ERROR: AXIVC manager device was not created: ${AXIVC_DEVICE}" >&2
    exit 1
fi

echo "AXIVC_READY device=${AXIVC_DEVICE}"
if [ "${AXIVC_MODULE_LOADED}" -eq 1 ] || [ -d /sys/module/axvisor ]; then
    echo "LINUX_AXIVC_READY device=${AXIVC_DEVICE} module=axvisor"
fi

if [ ! -x "${PERCEPTION_BIN}" ]; then
    echo "ERROR: perception program is missing or not executable: ${PERCEPTION_BIN}" >&2
    exit 1
fi
if [ ! -f "${MODEL_PATH}" ]; then
    echo "ERROR: RKNN model is missing: ${MODEL_PATH}" >&2
    exit 1
fi
if [ ! -f "${CALIBRATION_PATH}" ] || [ ! -f "${PICK_CONFIG_PATH}" ]; then
    echo "ERROR: robot calibration or pick configuration is missing" >&2
    echo "  calibration: ${CALIBRATION_PATH}" >&2
    echo "  pick config: ${PICK_CONFIG_PATH}" >&2
    exit 1
fi

if [ "${_AKA_DUAL_PICK_CI_ONCE:-0}" = "1" ]; then
    set -- --robot-ci-once --min-fps 15
else
    set --
fi

exec env LD_LIBRARY_PATH="${SCRIPT_DIR}/lib:${LD_LIBRARY_PATH:-}" \
    RKNN_CORE_MASK=0 \
    "${PERCEPTION_BIN}" "${MODEL_PATH}" "${CAMERA_INDEX}" \
    --transport ivc \
    --ivc-key "${IVC_KEY}" \
    --ivc-size "${IVC_SIZE}" \
    --calibration "${CALIBRATION_PATH}" \
    --pick-config "${PICK_CONFIG_PATH}" \
    --report-every 1 \
    --status-every 60 \
    --pipeline-heartbeat 0 \
    "$@"
