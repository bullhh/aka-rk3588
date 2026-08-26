#!/bin/sh

set -eu

SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" >/dev/null 2>&1 && pwd -P)
MIN_FPS=${MIN_FPS:-15}

exec "${SCRIPT_DIR}/run_dual_pick.sh" \
    --robot-ci-once \
    --min-fps "${MIN_FPS}" \
    "$@"
