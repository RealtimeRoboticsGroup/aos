#!/bin/bash
# --- begin runfiles.bash initialization v2 ---
# Copy-pasted from the Bazel Bash runfiles library v2.
set -uo pipefail; f=bazel_tools/tools/bash/runfiles/runfiles.bash
source "${RUNFILES_DIR:-/dev/null}/$f" 2>/dev/null || \
  source "$(grep -sm1 "^$f " "${RUNFILES_MANIFEST_FILE:-/dev/null}" | cut -f2- -d' ')" 2>/dev/null || \
  source "$0.runfiles/$f" 2>/dev/null || \
  source "$(grep -sm1 "^$f " "$0.runfiles_manifest" | cut -f2- -d' ')" 2>/dev/null || \
  source "$(grep -sm1 "^$f " "$0.exe.runfiles_manifest" | cut -f2- -d' ')" 2>/dev/null || \
  { echo>&2 "ERROR: cannot find $f"; exit 1; }; f=; set -e
# --- end runfiles.bash initialization v2 ---

export BAZEL_BINDIR=.

PRETTIER="$(rlocation _main/tools/lint/prettier_binary_/prettier_binary)"
if [[ -z "${PRETTIER}" ]]; then
  PRETTIER="$(rlocation aos/tools/lint/prettier_binary_/prettier_binary)"
fi

if [[ -z "${PRETTIER}" ]]; then
  echo >&2 "ERROR: cannot find prettier_binary in runfiles"
  exit 1
fi

exec "${PRETTIER}" "$@"
