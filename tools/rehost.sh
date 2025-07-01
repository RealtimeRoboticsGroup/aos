#!/bin/bash
#
# Rehosts a dependency to realtimeroboticsgroup.org/build-dependencies
# There's a gcp bucket behind this all, so just push to it.
set -xe

SOURCE="https://${1}"
echo "${SOURCE}" gs://austin-vpn-build-dependencies/$1

FILE=$(mktemp)
function cleanup {
  rm -rf "$FILE"
}

trap cleanup EXIT
trap cleanup SIGINT

curl -f -L "${SOURCE}" -o "${FILE}"

echo gs://austin-vpn-build-dependencies/$1

gsutil cp "${FILE}" "gs://austin-vpn-build-dependencies/$1"
