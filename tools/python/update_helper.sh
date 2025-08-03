#!/bin/bash

set -o errexit
set -o nounset
set -o pipefail

# --- begin runfiles.bash initialization v2 ---
# Copy-pasted from the Bazel Bash runfiles library v2.
set -uo pipefail; set +e; f=bazel_tools/tools/bash/runfiles/runfiles.bash
source "${RUNFILES_DIR:-/dev/null}/$f" 2>/dev/null || \
  source "$(grep -sm1 "^$f " "${RUNFILES_MANIFEST_FILE:-/dev/null}" | cut -f2- -d' ')" 2>/dev/null || \
  source "$0.runfiles/$f" 2>/dev/null || \
  source "$(grep -sm1 "^$f " "$0.runfiles_manifest" | cut -f2- -d' ')" 2>/dev/null || \
  source "$(grep -sm1 "^$f " "$0.exe.runfiles_manifest" | cut -f2- -d' ')" 2>/dev/null || \
  { echo>&2 "ERROR: cannot find $f"; exit 1; }; f=; set -e
# --- end runfiles.bash initialization v2 ---

DOCKERFILE="$(rlocation aos/tools/python/update_helper_files/Dockerfile)"
CONTEXT_DIR="$(dirname "${DOCKERFILE}")"
CONTAINER_TAG="pip-lock:${USER}"

# Build the container that has the bare minimum to run the various setup.py
# scripts from our dependencies.
docker build \
  --file="${DOCKERFILE}" \
  --tag="${CONTAINER_TAG}" \
  "${CONTEXT_DIR}"

perform_cleanup() {
  set -x
  # Restore permissions on the uv cache directory on exit.
  sudo chown -R "${USER}:${USER}" \
    "${HOME}"/.cache/uv \
    tools/python/requirements.lock.txt
}

trap perform_cleanup EXIT

# Run the actual update. The assumption here is that mounting the user's home
# directory is sufficient to allow the tool to run inside the container without
# any issues. I.e. the cache and the source tree are available in the
# container.
set -x
docker run \
  --rm \
  --tty \
  --env BUILD_WORKSPACE_DIRECTORY="${BUILD_WORKSPACE_DIRECTORY}" \
  --env UV_HTTP_TIMEOUT=300 \
  --env UV_CACHE_DIR=$HOME/.cache/uv \
  --workdir "${PWD}" \
  --volume "${HOME}:${HOME}" \
  "${CONTAINER_TAG}" \
  "$@"
