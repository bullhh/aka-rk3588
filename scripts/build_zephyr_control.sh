#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd -P)
AKA_ROOT=$(cd "${SCRIPT_DIR}/.." && pwd -P)
TGOSIMAGES_REPO_URL="${TGOSIMAGES_REPO_URL:-https://github.com/rcore-os/tgosimages.git}"
TGOSIMAGES_DIR="${TGOSIMAGES_DIR:-}"
IVC_SDK_DIR="${IVC_SDK_DIR:-}"
FORWARD_ARGS=()

usage() {
    cat <<'EOF'
Build the aka-rk3588 Zephyr robot controller with TGOSImages.

Usage:
  scripts/build_zephyr_control.sh [--tgosimages-dir <path>]
      [--ivc-sdk-dir <path>] [Zephyr options]

Resolution order:
  1. --tgosimages-dir
  2. TGOSIMAGES_DIR
  3. sibling directory ../tgosimages
  4. clone the current TGOSImages default branch into ../tgosimages

An existing TGOSImages checkout is never pulled, switched, or cleaned.
Unrecognized options are passed to TGOSImages scripts/os/zephyr.sh.
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --tgosimages-dir)
            [[ $# -ge 2 ]] || { echo "ERROR: --tgosimages-dir needs a path" >&2; exit 2; }
            TGOSIMAGES_DIR="$2"
            shift 2
            ;;
        --ivc-sdk-dir)
            [[ $# -ge 2 ]] || { echo "ERROR: --ivc-sdk-dir needs a path" >&2; exit 2; }
            IVC_SDK_DIR="$2"
            shift 2
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            FORWARD_ARGS+=("$1")
            shift
            ;;
    esac
done

if [[ -z "${TGOSIMAGES_DIR}" ]]; then
    TGOSIMAGES_DIR="${AKA_ROOT}/../tgosimages"
fi

if [[ ! -d "${TGOSIMAGES_DIR}/.git" ]]; then
    if [[ -e "${TGOSIMAGES_DIR}" ]]; then
        echo "ERROR: ${TGOSIMAGES_DIR} exists but is not a Git checkout" >&2
        exit 1
    fi
    echo "TGOSImages is absent; cloning ${TGOSIMAGES_REPO_URL} to ${TGOSIMAGES_DIR}"
    git clone "${TGOSIMAGES_REPO_URL}" "${TGOSIMAGES_DIR}"
fi

TGOSIMAGES_DIR=$(cd "${TGOSIMAGES_DIR}" && pwd -P)
ENTRY="${TGOSIMAGES_DIR}/scripts/apps/aka-rk3588-zephyr.sh"
if [[ ! -x "${ENTRY}" ]]; then
    echo "ERROR: TGOSImages entry script is missing or not executable: ${ENTRY}" >&2
    echo "The selected TGOSImages checkout does not yet support the aka-rk3588 external app." >&2
    exit 1
fi

echo "AKA_RK3588_DIR=${AKA_ROOT}"
echo "TGOSIMAGES_DIR=${TGOSIMAGES_DIR}"
echo "TGOSIMAGES_COMMIT=$(git -C "${TGOSIMAGES_DIR}" rev-parse --short HEAD)"
if [[ -n "$(git -C "${TGOSIMAGES_DIR}" status --short)" ]]; then
    echo "TGOSIMAGES_WORKTREE=dirty (using local files without changing them)"
else
    echo "TGOSIMAGES_WORKTREE=clean"
fi

ENTRY_ARGS=(--aka-dir "${AKA_ROOT}")
if [[ -n "${IVC_SDK_DIR}" ]]; then
    ENTRY_ARGS+=(--ivc-sdk-dir "${IVC_SDK_DIR}")
fi
exec "${ENTRY}" "${ENTRY_ARGS[@]}" "${FORWARD_ARGS[@]}"
