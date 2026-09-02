#!/bin/sh

set -eu

# Compatibility entry point. AXIVC preparation is implemented once in
# run_dual_pick.sh so Linux and StarryOS use the same startup path.
SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" >/dev/null 2>&1 && pwd -P)
exec "${SCRIPT_DIR}/run_dual_pick.sh" "$@"
