#!/bin/sh

set -eu

SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" >/dev/null 2>&1 && pwd -P)
PERCEPTION_BIN=${PERCEPTION_BIN:-${SCRIPT_DIR}/build-ivc-native-linux/tennis-perception}
MODEL_PATH=${MODEL_PATH:-${SCRIPT_DIR}/models/tennis.rknn}
CAMERA_INDEX=${CAMERA_INDEX:-0}
RKNN_CORE_MASK=${RKNN_CORE_MASK:-0}
REPORT_EVERY=${REPORT_EVERY:-1}
STATUS_EVERY=${STATUS_EVERY:-60}
PIPELINE_HEARTBEAT=${PIPELINE_HEARTBEAT:-0}

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
    --report-every "${REPORT_EVERY}" \
    --status-every "${STATUS_EVERY}" \
    --pipeline-heartbeat "${PIPELINE_HEARTBEAT}" \
    "$@"
