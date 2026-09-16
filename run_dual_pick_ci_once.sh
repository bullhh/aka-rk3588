#!/bin/sh

set -eu

if [ "$#" -ne 2 ] || [ "$1" != "--min-fps" ]; then
    echo "Usage: run_dual_pick_ci_once.sh --min-fps <positive FPS>" >&2
    exit 2
fi

if ! awk -v value="$2" 'BEGIN {
    exit !(value ~ /^[0-9]+([.][0-9][0-9]?)?$/ && value + 0 > 0 && value + 0 <= 1000)
}'; then
    echo "ERROR: --min-fps must be in (0, 1000] with at most two decimal places" >&2
    exit 2
fi

SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" >/dev/null 2>&1 && pwd -P)
_AKA_DUAL_PICK_CI_ONCE=1
_AKA_DUAL_PICK_CI_MIN_FPS=$2
export _AKA_DUAL_PICK_CI_ONCE _AKA_DUAL_PICK_CI_MIN_FPS
# Preserve the application's status across the POSIX pipeline without pipefail
# or temporary files. Only publisher stdout/stderr enters this checker; guest
# Zephyr UART output is deliberately not part of the acceptance contract.
set +e
{
    "${SCRIPT_DIR}/run_dual_pick.sh"
    printf '\nDUAL_PICK_PROCESS_EXIT status=%d\n' "$?"
} 2>&1 | awk -v minimum="${_AKA_DUAL_PICK_CI_MIN_FPS}" '
function field(key, i, pair) {
    for (i = 2; i <= NF; i++) {
        split($i, pair, "=")
        if (pair[1] == key) return pair[2]
    }
    return ""
}
function numeric(value) { return value ~ /^[0-9]+([.][0-9]+)?$/ }
{ sub(/\r$/, ""); print; fflush() }
/^ROBOT_CONFIG_APPLIED / {
    session = field("session")
    if (!numeric(session) || session + 0 == 0 || configured) invalid = 1
    configured = 1
}
/^ROBOT_CONTROL_DONE / {
    cycles = field("cycles")
    if (!configured || control || done || field("session") != session ||
        field("status") != "ok" || !numeric(cycles) || cycles + 0 < 1 ||
        field("checks") != "7") invalid = 1
    control = 1
}
/^STARRY_ROBOT_CI_PERF_WINDOW / {
    fps = field("effective_fps")
    threshold = field("threshold")
    elapsed = field("elapsed_ms")
    if (done || windows >= 2 || field("index") != ((windows + 1) "/2") ||
        !numeric(fps) || fps + 0 < minimum + 0 ||
        !numeric(threshold) || threshold + 0 != minimum + 0 ||
        !numeric(elapsed) || elapsed + 0 < 10000) invalid = 1
    windows++
}
/^STARRY_ROBOT_CI_DONE / {
    duration = field("duration_ms")
    if (done || !control || windows != 2 || field("perf") != "pass" ||
        !numeric(duration) || duration + 0 < 62000) invalid = 1
    done = 1
}
/^DUAL_PICK_PROCESS_EXIT status=/ {
    status = field("status")
    exited = numeric(status)
}
END {
    if (!exited) exit 1
    if (status + 0 != 0) exit status + 0
    if (invalid || windows != 2 || !done || !configured || !control) {
        print "DUAL_PICK_CI_CHECK_FAILED reason=publisher-result-incomplete-or-invalid"
        exit 1
    }
    print "DUAL_PICK_CI_CHECK_PASS min_fps=" minimum
}'
