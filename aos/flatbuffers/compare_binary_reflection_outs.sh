#!/bin/bash

# This test script takes in 4 files as arguments, and verifies that they are all
# binary identical. See the corresponding Bazel target for the exact args passed
# in.
#
# The goal here is to verify that the compiled reflection output from all 4
# flavors of the same flatbuffers schema are bytewise identical. The 4 flavors
# of .fbs files are:
# 1. Local, checked-in
# 2. Local, generated
# 3. External, checked-in
# 4. External, generated
#
# This guarantees that flatc successfully strips out build system and repository
# specific components (such as relative paths, workspace names, etc.), rendering
# the compiled binary reflection output completely independent of the repository
# context in which it is built.

set -e

FILE1="$1"
FILE2="$2"
FILE3="$3"
FILE4="$4"

echo "Comparing bfbs hashes:"
sha256sum "$FILE1" "$FILE2" "$FILE3" "$FILE4"

# Check that all files are binary identical
cmp "$FILE1" "$FILE2"
cmp "$FILE1" "$FILE3"
cmp "$FILE1" "$FILE4"

echo "PASS: All 4 binary reflection schemas (.bfbs) are identical!"
