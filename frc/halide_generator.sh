#!/bin/bash
# We need to build code linked against Halide. This means we need to use a
# compatible ABI. This means we need to use libstdc++, not libc++ like our main
# toolchains are set up for.
#
# Rebuilding Halide itself is only moderately annoying. However, it needs to
# link against LLVM, which is a much bigger pain to rebuild with libc++.
#
# To deal with this problem, this script runs clang hermetically on the
# appropriate sources.
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
BINARY="$1"
SOURCE="$2"
HALIDE="$(rlocation halide_k8)"
SYSROOT="$(rlocation amd64_debian_sysroot)"
LLVM_TOOLCHAIN="$(dirname "$(dirname "$(rlocation llvm_k8/bin/clang)")")"
TARGET=x86_64-unknown-linux-gnu
MULTIARCH=x86_64-linux-gnu
export LD_LIBRARY_PATH=external/llvm_toolchain/llvm/lib/
"${LLVM_TOOLCHAIN}/bin/clang++" \
  -fcolor-diagnostics \
  -I"${HALIDE}/include" \
  -nostdinc \
  -isystem"${SYSROOT}/usr/include/c++/12" \
  -isystem"${SYSROOT}/usr/include/${MULTIARCH}/c++/12" \
  -isystem"${SYSROOT}/usr/include/c++/7/backward" \
  -isystem"${LLVM_TOOLCHAIN}/lib/clang/19/include" \
  -isystem"${SYSROOT}/usr/include/${MULTIARCH}" \
  -isystem"${SYSROOT}/usr/include" \
  -isystem"${SYSROOT}/include" \
  "--sysroot=${SYSROOT}" \
  -resource-dir "${LLVM_TOOLCHAIN}/lib/clang/19" \
  -target "${TARGET}" \
  -fuse-ld=lld \
  -L"${LLVM_TOOLCHAIN}/lib" \
  -L"${SYSROOT}/usr/lib" \
  -L"${SYSROOT}/usr/lib/gcc/${MULTIARCH}/12" \
  -L"${SYSROOT}/usr/lib/${MULTIARCH}" \
  "${HALIDE}/lib/libHalide.a" \
  -lstdc++ -lpthread -ldl -lm -lz \
  -std=gnu++20 \
  "${SOURCE}" \
  "${HALIDE}/share/Halide/tools/GenGen.cpp" \
  -ggdb3 \
  -o "${BINARY}"
