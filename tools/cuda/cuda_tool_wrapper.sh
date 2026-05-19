#!/bin/bash
# Generic wrapper around the CUDA host tools (ptxas, fatbinary,
# nvlink, bin2c) shipped in the aarch64 Orin sysroot. All four are
# x86_64-linux ELF binaries; clang invokes them by name from the
# synthetic CUDA root at `tools/cuda/vm/_cache/cuda_root/bin/`, where
# every entry is a copy of *this* script. The actual tool dispatched
# is determined by `basename "$0"`.
#
#   * On Linux: exec() the real x86_64-linux binary out of the
#     `@arm64_debian_sysroot` repo unchanged.
#
#   * On Darwin: spawn a one-shot Apple Virtualization framework
#     microvm via `vfkit`, share the relevant directories into it,
#     register Rosetta as the binfmt handler for x86_64 ELFs, run the
#     requested tool through Rosetta, and tear the VM down.
#
# No daemon, no persistent VM, no docker. ~10s of Rosetta-translation
# cost the first time a given tool runs in each invocation; the cache
# does not outlive the VM.

set -euo pipefail

TOOL_NAME="$(basename "$0")"
EXECROOT="$PWD"
CUDA_BIN_REL="${AOS_CUDA_BIN_REL:-/usr/local/cuda-12.6/bin}"
# fatbinary in the aarch64 sysroot is named per-target.
case "$TOOL_NAME" in
  fatbinary) TOOL_BIN="aarch64-unknown-linux-gnu-fatbinary" ;;
  *)         TOOL_BIN="$TOOL_NAME" ;;
esac
TOOL_REL="${CUDA_BIN_REL}/${TOOL_BIN}"

# Inside a bazel darwin-sandbox, `external/<repo>+` is a regular
# directory whose individual *files* are symlinks back into the real
# output_base. To get the canonical sysroot path we read just the
# first symlink hop (not the full realpath chain, which would chase
# through internal symlinks within the sysroot) and trim everything
# after the `external/<repo>+` segment.
sysroot_root() {
  local probe="$1" repo="$2"
  if [[ ! -e "$probe" ]]; then
    echo "${EXECROOT}/external/${repo}"
    return
  fi
  /usr/bin/python3 - "$probe" "$repo" <<'PY'
import os, sys, re
target = os.readlink(sys.argv[1]) if os.path.islink(sys.argv[1]) \
                                  else os.path.realpath(sys.argv[1])
m = re.match(r'(.+/external/' + re.escape(sys.argv[2]) + r')(/|$)', target)
if not m:
    sys.exit(f"sysroot_root: {target!r} doesn't contain external/{sys.argv[2]}")
print(m.group(1))
PY
}
ROOTFS_ARM64="${AOS_ARM64_ROOTFS:-$(sysroot_root \
  "${EXECROOT}/external/arm64_debian_sysroot+${CUDA_BIN_REL}/ptxas" \
  "arm64_debian_sysroot+")}"
ROOTFS_AMD64="${AOS_AMD64_ROOTFS:-$(sysroot_root \
  "${EXECROOT}/external/amd64_debian_sysroot+/lib64/ld-linux-x86-64.so.2" \
  "amd64_debian_sysroot+")}"
TOOL="${ROOTFS_ARM64}${TOOL_REL}"

if [[ "$(uname -s)" != "Darwin" ]]; then
  exec "$TOOL" "$@"
fi

# Darwin path: boot a vfkit microvm with Rosetta and run ptxas inside.
# Bazel sandboxed actions inherit a stripped PATH; restore Homebrew so
# vfkit is reachable.
export PATH="/opt/homebrew/bin:/opt/homebrew/sbin:/usr/local/bin:/usr/bin:/bin:/usr/sbin:/sbin:${PATH:-}"

VFKIT="${VFKIT:-/opt/homebrew/bin/vfkit}"
if [[ ! -x "$VFKIT" ]]; then
  echo "cuda_tool_wrapper.sh: vfkit not found at $VFKIT" >&2
  echo "cuda_tool_wrapper.sh: install with: brew install vfkit" >&2
  exit 1
fi

# Locate the kernel + initramfs. These are bazel-generated outputs
# (via `tools/cuda:vfkit_kernel_image` / `:vfkit_initramfs`), and end
# up in the action sandbox under `bazel-out/<config>/bin/tools/cuda/
# vm/_cache/`. The exact config segment varies per build, so glob it.
# Honour env-var overrides for local testing.
find_one() {
  local name="$1"; shift
  for p in "$@"; do
    [[ -f "$p" ]] && { printf '%s\n' "$p"; return 0; }
  done
  echo "cuda_tool_wrapper.sh: $name not found; tried:" >&2
  for p in "$@"; do echo "  $p" >&2; done
  return 1
}

# `nullglob` so unmatched bazel-out/* patterns expand to nothing.
shopt -s nullglob
kernel_candidates=(
  ${AOS_VFKIT_KERNEL:+"$AOS_VFKIT_KERNEL"}
  bazel-out/*/bin/tools/cuda/vm/_cache/Image
  tools/cuda/vm/_cache/Image
)
initrd_candidates=(
  ${AOS_VFKIT_INITRD:+"$AOS_VFKIT_INITRD"}
  bazel-out/*/bin/tools/cuda/vm/_cache/initramfs-aos.cpio.gz
  tools/cuda/vm/_cache/initramfs-aos.cpio.gz
)
shopt -u nullglob

KERNEL=$(find_one kernel "${kernel_candidates[@]}") || exit 1
INITRD=$(find_one initramfs "${initrd_candidates[@]}") || exit 1
KERNEL="$(cd "$(dirname "$KERNEL")" && pwd)/$(basename "$KERNEL")"
INITRD="$(cd "$(dirname "$INITRD")" && pwd)/$(basename "$INITRD")"

# Each invocation gets its own scratch dir for the workdir share; the
# VM reads argv/env from it and writes stdout/stderr/exit back.
WORKDIR="$(mktemp -d -t aos-cuda)"
# Keep the workdir around on failure so the user can poke at
# stdout/stderr/console.log; on success it's cleaned up.
trap '[[ "${KEEP_WORKDIR:-}" = 1 ]] || rm -rf "$WORKDIR"' EXIT

cat >"$WORKDIR/env" <<EOF
AMD64=$ROOTFS_AMD64
ARM64=$ROOTFS_ARM64
CWD=$EXECROOT
TOOL=$TOOL
EOF
printf '%s\0' "$@" > "$WORKDIR/argv"

# `printk.time=1` always-on so the captured-on-failure console.log has
# timestamps. Set `AOS_VFKIT_VERBOSE=1` to also drop `quiet` and dump
# console.log on success — handy for boot-time profiling, e.g.
# `AOS_VFKIT_VERBOSE=1 tools/bazel build --action_env=AOS_VFKIT_VERBOSE \
#    --config=arm64 //frc/orin:some_cuda_target` after `touch`-ing one
# of its CUDA sources.
KCMDLINE='console=hvc0 panic=3 random.trust_cpu=on rng_core.default_quality=1000 printk.time=1'
if [[ "${AOS_VFKIT_VERBOSE:-}" = 1 ]]; then
  KCMDLINE="$KCMDLINE loglevel=8 ignore_loglevel initcall_debug"
else
  KCMDLINE="$KCMDLINE quiet"
fi

# clang writes intermediate `.s` files into the host's /tmp (macOS
# resolves /tmp to /private/tmp); make that visible inside the VM.
"$VFKIT" \
  --cpus 2 --memory 1024 \
  --kernel "$KERNEL" \
  --initrd "$INITRD" \
  --kernel-cmdline "$KCMDLINE" \
  --device virtio-rng \
  --device "virtio-serial,logFilePath=$WORKDIR/console.log" \
  --device "rosetta,mountTag=rosetta" \
  --device "virtio-fs,sharedDir=$WORKDIR,mountTag=workdir" \
  --device "virtio-fs,sharedDir=$ROOTFS_AMD64,mountTag=amd64_sysroot" \
  --device "virtio-fs,sharedDir=$ROOTFS_ARM64,mountTag=arm64_sysroot" \
  --device "virtio-fs,sharedDir=$EXECROOT,mountTag=execroot" \
  --device "virtio-fs,sharedDir=/private/tmp,mountTag=host_tmp" \
  >"$WORKDIR/vfkit.log" 2>&1

# Forward whatever ptxas wrote.
[[ -s "$WORKDIR/stdout" ]] && cat "$WORKDIR/stdout"
[[ -s "$WORKDIR/stderr" ]] && cat "$WORKDIR/stderr" >&2

rc=
[[ -f "$WORKDIR/exit" ]] && rc=$(cat "$WORKDIR/exit")

# Echo the captured kernel console when explicitly requested (boot
# profiling), independent of exit status.
if [[ "${AOS_VFKIT_VERBOSE:-}" = 1 ]]; then
  {
    echo "cuda_tool_wrapper.sh: --- console.log (verbose) ---"
    cat "$WORKDIR/console.log" 2>/dev/null || true
  } >&2
fi

# Surface diagnostics on failure so the user isn't left guessing why
# the VM didn't return a normal exit code.
if [[ -z "$rc" || "$rc" -ne 0 ]]; then
  KEEP_WORKDIR=1
  {
    echo "cuda_tool_wrapper.sh: $TOOL_NAME ($TOOL) exit=${rc:-MISSING}"
    echo "cuda_tool_wrapper.sh: workdir kept at $WORKDIR"
    echo "cuda_tool_wrapper.sh: argv (NUL-separated):"
    tr '\0' '\n' < "$WORKDIR/argv" 2>/dev/null | sed 's/^/  /' || true
    echo "cuda_tool_wrapper.sh: --- vfkit.log ---"
    cat "$WORKDIR/vfkit.log" 2>/dev/null || true
    echo "cuda_tool_wrapper.sh: --- console.log ---"
    cat "$WORKDIR/console.log" 2>/dev/null || true
  } >&2
fi
exit "${rc:-127}"
