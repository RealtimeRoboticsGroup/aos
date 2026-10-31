#!/bin/bash

# Copies every generated bundle file into the checked-in golden/ directory,
# overwriting whatever's there. Run via `bazel run` so that
# BUILD_WORKSPACE_DIRECTORY points at the source tree.

set -o errexit -o nounset -o pipefail

OUTPUT_DIR="${BUILD_WORKSPACE_DIRECTORY}/tools/foxglove/ts_generator/test/golden"
mkdir -p "${OUTPUT_DIR}"
for src in "$@"; do
    dest="${OUTPUT_DIR}/$(basename "${src}")"
    # Bazel marks files in bazel-bin read-only and our previous copy of them
    # inherits that, so make the destination writable before overwriting it.
    chmod u+w "${dest}" 2>/dev/null || true
    cp "${src}" "${dest}"
    chmod 644 "${dest}"
done
