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

GAZELLE_BIN="$(rlocation aos/gazelle)"
if [[ -z "${GAZELLE_BIN}" ]]; then
  GAZELLE_BIN="$(rlocation _main/gazelle)"
fi

cd "${BUILD_WORKSPACE_DIRECTORY}"

# Clear out the go_deps.bzl file so that gazelle won't hesitate to update
# it. Without this step gazelle would never try to remove a dependency.
cat > go_deps.bzl <<EOF
def go_dependencies():
    pass
EOF

export GOPROXY="https://proxy.golang.org,direct"
export GOSUMDB="sum.golang.org"

"${GAZELLE_BIN}" update-repos \
    -from_file=go.mod \
    -to_macro=go_deps.bzl%go_dependencies \
    -prune
