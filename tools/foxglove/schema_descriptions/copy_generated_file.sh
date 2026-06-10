#!/bin/bash

# Copies the freshly generated schema file back into the source tree so it can be checked in as
# the golden output for the comparison test.
#
# This tool lives in the AOS workspace but is also consumed by downstream workspaces (e.g. the
# BRT workspace, which pulls AOS in as a `local_repository`). When invoked, `BUILD_WORKSPACE_DIRECTORY`
# points at whichever workspace `bazel run` was issued from, so we try both candidate locations.

set -euo pipefail

CANDIDATES=(
    "${BUILD_WORKSPACE_DIRECTORY}/tools/foxglove/schema_descriptions/test_types.expected.generated.ts"
    "${BUILD_WORKSPACE_DIRECTORY}/third_party/aos/tools/foxglove/schema_descriptions/test_types.expected.generated.ts"
)

for candidate in "${CANDIDATES[@]}"; do
    if [[ -f "${candidate}" ]]; then
        cp "$1" "${candidate}"
        chmod 644 "${candidate}"
        echo "Updated ${candidate}"
        exit 0
    fi
done

echo "ERROR: could not find the golden file under \$BUILD_WORKSPACE_DIRECTORY=${BUILD_WORKSPACE_DIRECTORY}" >&2
echo "Tried:" >&2
printf '  %s\n' "${CANDIDATES[@]}" >&2
exit 1
