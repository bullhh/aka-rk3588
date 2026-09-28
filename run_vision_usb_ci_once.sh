#!/bin/sh
# One bounded vision + FT232R USB loopback CI flow (virtual branch).
#
# Runs real UVC capture / JPEG decode / RKNN inference for two 10s windows
# concurrently with a bounded FT232R (0403:6001) binary loopback cadence, then
# emits a single [VISION_USB_CI] RESULT=PASS only when the application's final
# APPLICATION_PASS marker and every contract field are present and consistent.
# There is no retry and no legacy ROBOT_CI success marker.
#
# Usage:
#   ./run_vision_usb_ci_once.sh [min_fps]
#
# Environment:
#   MODEL_PATH   override the model path (default models/tennis.rknn)
#   UVC_INDEX    UVC device index (default 0)
#   FT232_SERIAL optional serial selector for multi-adapter boards
#   RKNN_CORE_MASK NPU core mask (default 0)

set -u

SCRIPT_DIR="$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)"
# Relocated CI deployments must use their own bundled RKNN runtime.
LD_LIBRARY_PATH="${SCRIPT_DIR}/lib:${LD_LIBRARY_PATH:-}"
export LD_LIBRARY_PATH

MIN_FPS="${1:-28.0}"
MODEL_PATH="${MODEL_PATH:-${SCRIPT_DIR}/models/tennis.rknn}"
UVC_INDEX="${UVC_INDEX:-0}"
# Fixed CI floor shared with the application: 160 successful exchanges per 10s
# window at the default 20 Hz cadence, i.e. 320 over the two windows.
WINDOW_TX_FLOOR=160
TOTAL_TX_FLOOR=320
# Must stay in sync with vision_usb_ci.cpp: a window tolerates up to 3 bad
# MJPEG frames, so the two-window total is 6.
WINDOW_JPEG_MAX=3
TOTAL_JPEG_MAX=6

if [ "$#" -gt 1 ] || ! awk -v value="${MIN_FPS}" 'BEGIN {
    exit !(value ~ /^[0-9]+([.][0-9]+)?$/ && value + 0 > 0 && value + 0 <= 1000)
}'; then
    echo "[VISION_USB_CI] RESULT=FAIL reason=invalid_min_fps"
    exit 2
fi

if [ ! -x "${SCRIPT_DIR}/build/tennis" ]; then
    echo "[VISION_USB_CI] RESULT=FAIL reason=missing_binary path=${SCRIPT_DIR}/build/tennis"
    exit 1
fi
if [ ! -f "${MODEL_PATH}" ]; then
    echo "[VISION_USB_CI] RESULT=FAIL reason=missing_model path=${MODEL_PATH}"
    exit 1
fi

RKNN_CORE_MASK="${RKNN_CORE_MASK:-0}"
export RKNN_CORE_MASK

app_status=0
{
    cd "${SCRIPT_DIR}" || exit 1
    "${SCRIPT_DIR}/build/tennis" vision-usb-ci \
        "${MODEL_PATH}" "${MIN_FPS}" "${UVC_INDEX}"
    printf '\nVISION_USB_CI_PROCESS_EXIT status=%d\n' "$?"
} 2>&1 | awk -v minimum="${MIN_FPS}" -v window_floor="${WINDOW_TX_FLOOR}" \
    -v total_floor="${TOTAL_TX_FLOOR}" -v jpeg_window_max="${WINDOW_JPEG_MAX}" \
    -v jpeg_total_max="${TOTAL_JPEG_MAX}" '
function field(key, i, pair) {
    for (i = 2; i <= NF; i++) {
        split($i, pair, "=")
        if (pair[1] == key) return pair[2]
    }
    return ""
}
function numeric(value) { return value ~ /^[0-9]+([.][0-9]+)?$/ }
{ sub(/\r$/, ""); print; fflush() }
# The DEVICE line must name exactly one transport so the coverage is explicit.
/^\[VISION_USB_CI\] DEVICE / {
    devices++
    if (index($0, "transport=usb") == 0 && index($0, "transport=tty") == 0)
        invalid = 1
}
# Per-window evidence: exactly two windows, each at least 10s with real work.
/^\[VISION_USB_CI\] PERF_WINDOW / {
    windows++
    if (field("index") != windows "/2") invalid = 1
    elapsed = field("elapsed_s")
    processed = field("processed")
    jerr = field("jpeg_errors")
    if (!numeric(elapsed) || elapsed + 0 < 10 ||
        processed !~ /^[0-9]+$/ || processed + 0 <= 0 ||
        !numeric(jerr) || jerr + 0 > jpeg_window_max + 0) invalid = 1
    window_jpeg += jerr + 0
}
# Each window must also carry enough successful loopback exchanges.
/^\[VISION_USB_CI\] LOOPBACK_WINDOW / {
    loopbacks++
    if (field("index") != loopbacks "/2") invalid = 1
    ltx = field("tx")
    lerr = field("errors")
    if (ltx !~ /^[0-9]+$/ || ltx + 0 < window_floor ||
        lerr !~ /^[0-9]+$/ || lerr + 0 != 0) invalid = 1
}
# Only this final marker can establish success, and only once.
/^\[VISION_USB_CI\] APPLICATION_PASS / {
    markers++
    fps = field("effective_fps")
    elapsed = field("elapsed_s")
    processed = field("processed")
    threshold = field("min_fps")
    tx = field("loopback_tx")
    errors = field("loopback_errors")
    mintx = field("loopback_min_tx")
    jsum = field("jpeg_errors")
    if (markers > 1 || field("windows") != "2" || field("pause_resume") != "1" ||
        !numeric(threshold) || threshold + 0 != minimum + 0 ||
        !numeric(fps) || fps + 0 < minimum + 0 ||
        !numeric(elapsed) || elapsed + 0 < 20 ||
        processed !~ /^[0-9]+$/ || processed + 0 <= 0 ||
        tx !~ /^[0-9]+$/ || tx + 0 < total_floor ||
        errors !~ /^[0-9]+$/ || errors + 0 != 0 ||
        mintx !~ /^[0-9]+$/ || mintx + 0 < total_floor ||
        tx + 0 < mintx + 0 ||
        !numeric(jsum) || jsum + 0 < 0 || jsum + 0 > jpeg_total_max + 0) invalid = 1
    pass_jpeg = jsum
}
# The non-final summary must also carry the per-window total so a truncated run
# or a summary/PASS mismatch cannot pass.
/^\[VISION_USB_CI\] PERF_SUMMARY / {
    summaries++
    summary_jpeg = field("jpeg_errors")
    if (!numeric(summary_jpeg)) invalid = 1
}
/^VISION_USB_CI_PROCESS_EXIT / {
    if (exited++) invalid = 1
    status = field("status")
}
END {
    if (!exited || !numeric(status) || status + 0 != 0 || invalid ||
        markers != 1 || windows != 2 || loopbacks != 2 || devices != 1 ||
        summaries != 1 || !numeric(pass_jpeg) || !numeric(summary_jpeg) ||
        pass_jpeg + 0 != window_jpeg + 0 || summary_jpeg + 0 != window_jpeg + 0)
        exit 1
    exit 0
}' || app_status=$?

if [ "${app_status}" -eq 0 ]; then
    echo "[VISION_USB_CI] RESULT=PASS attempts=1"
    exit 0
fi

echo "[VISION_USB_CI] RESULT=FAIL attempts=1"
exit "${app_status}"
