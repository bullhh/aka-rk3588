#!/bin/sh

set -eu

if [ "$#" -ne 0 ]; then
    echo "ERROR: run_dual_pick_ci_once.sh does not accept arguments" >&2
    exit 2
fi

SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" >/dev/null 2>&1 && pwd -P)
_AKA_DUAL_PICK_CI_ONCE=1
export _AKA_DUAL_PICK_CI_ONCE
exec "${SCRIPT_DIR}/run_dual_pick.sh"
