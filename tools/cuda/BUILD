"""Hermetic CUDA toolchains for rules_cuda.

The CUDA bits we need (`ptxas`, `fatbinary`, libdevice, headers) ship inside
the per-platform sysroot repos, so we just point rules_cuda's clang
toolchain at those file labels. Two toolchains are registered, gated by
`@platforms//cpu`, so Bazel selects the right one for `--platforms`.

Compared to the old approach of stitching symlinks into a hermetic repo,
this keeps every input as a regular Bazel artifact, which is required for
remote execution to work.
"""

load("@platforms//host:constraints.bzl", "HOST_CONSTRAINTS")
load("@rules_cuda//cuda:defs.bzl", "cuda_toolchain", "cuda_toolchain_config_clang", "cuda_toolkit_info")
load("@rules_python//python:defs.bzl", "py_binary")
load(":synth_root.bzl", "synth_cuda_root")

package(default_visibility = ["//visibility:public"])

# Host-platform discriminator. The synth-root and vfkit-cache pieces
# only matter on macOS; on Linux clang dispatches the real x86_64 host
# tools directly out of the aarch64 sysroot, so we skip the wrapper
# script and the entire VM build pipeline.
_HOST_IS_DARWIN = "@platforms//os:osx" in HOST_CONSTRAINTS

# ---------------------------------------------------------------------------
# x86_64: Debian sysroot ships CUDA 11.8 under /usr/lib/cuda and /usr/bin.
# ---------------------------------------------------------------------------

filegroup(
    name = "x86_64_files",
    srcs = [
        "@amd64_debian_sysroot//:usr/bin/ptxas",
        "@amd64_debian_sysroot//:usr/lib/cuda/bin/x86_64-unknown-linux-gnu-fatbinary",
        "@amd64_debian_sysroot//:usr/lib/cuda/nvvm/libdevice/libdevice.10.bc",
    ],
)

cuda_toolkit_info(
    name = "x86_64_toolkit_info",
    bin2c = "@rules_cuda//cuda/dummy:bin2c",
    fatbinary = "@amd64_debian_sysroot//:usr/lib/cuda/bin/x86_64-unknown-linux-gnu-fatbinary",
    libdevice = "@amd64_debian_sysroot//:usr/lib/cuda/nvvm/libdevice/libdevice.10.bc",
    link_stub = "@rules_cuda//cuda/dummy:link.stub",
    nvlink = "@rules_cuda//cuda/dummy:nvlink",
    path = "",
    ptxas = "@amd64_debian_sysroot//:usr/bin/ptxas",
    version = "11.8",
)

cuda_toolchain_config_clang(
    name = "x86_64_clang_config",
    cuda_toolkit = ":x86_64_toolkit_info",
    toolchain_identifier = "clang",
)

cuda_toolchain(
    name = "x86_64_clang",
    compiler_files = [":x86_64_files"],
    compiler_use_cc_toolchain = True,
    toolchain_config = ":x86_64_clang_config",
)

toolchain(
    name = "x86_64_clang_toolchain",
    target_compatible_with = [
        "@platforms//cpu:x86_64",
        "//tools/platforms/gpu:nvidia",
    ],
    target_settings = [
        "@rules_cuda//cuda:is_enabled",
        "@rules_cuda//cuda:compiler_is_clang",
    ],
    toolchain = ":x86_64_clang",
    toolchain_type = "@rules_cuda//cuda:toolchain_type",
)

# ---------------------------------------------------------------------------
# aarch64: Yocto sysroot ships CUDA 12.6 under /usr/local/cuda-12.6.
# ---------------------------------------------------------------------------

# Synthetic CUDA root used by the aarch64 toolchain. `cuda_tool_wrapper.sh`
# is copied to `bin/{ptxas,fatbinary,nvlink,bin2c}` so clang's
# `--cuda-path=` (which rules_cuda derives from libdevice's location)
# resolves every tool back to a single wrapper that dispatches by
# basename. On Linux the wrapper exec()s the real x86_64 binary out of
# the arm64 sysroot; on Darwin it spawns a one-shot vfkit microvm and
# runs the tool through Rosetta. The amd64 sysroot is an input because
# the Darwin path needs its x86_64 libc + dynamic loader inside the VM.
synth_cuda_root(
    name = "aarch64_synth_root",
    bin2c = "synth_root/aarch64/bin/bin2c",
    cuda_root_subdir = "usr/local/cuda-12.6",
    fatbinary = "synth_root/aarch64/bin/fatbinary",
    libdevice = "synth_root/aarch64/nvvm/libdevice/libdevice.10.bc",
    nvlink = "synth_root/aarch64/bin/nvlink",
    out_subdir = "synth_root/aarch64",
    ptxas = "synth_root/aarch64/bin/ptxas",
    src_files = "@arm64_debian_sysroot//:cuda_12_6_root",
    # Darwin needs the vfkit/Rosetta dispatcher in front of every host
    # tool. On Linux the synth root just hosts a single rename
    # symlink for fatbinary; bin/ptxas, bin/nvlink, and bin/bin2c
    # symlink straight to the real x86_64 binaries.
    wrapper = "cuda_tool_wrapper.sh" if _HOST_IS_DARWIN else None,
)

# ---------------------------------------------------------------------------
# vfkit boot artifacts. We pull Alpine's raw aarch64 netboot files from
# the mirror and assemble the kernel `Image` + repacked initramfs at
# build time. The mirror only carries Alpine's untouched bytes (rehosted
# via `tools/rehost.sh`); the assembly is fully deterministic from those
# inputs plus `tools/cuda/vm/init`.
# ---------------------------------------------------------------------------

# Alpine kernel version baked into the modloop layout. Bump alongside
# the Alpine pins in MODULE.bazel and keep tools/cuda/vm/init's
# EXPECTED_KERNEL_VERSION in sync.
_ALPINE_KERNEL_VERSION = "6.6.134-0-virt"

py_binary(
    name = "extract_kernel_image",
    srcs = ["vm/extract_kernel_image.py"],
    visibility = ["//visibility:private"],
)

py_binary(
    name = "repack_initramfs",
    srcs = ["vm/repack_initramfs.py"],
    visibility = ["//visibility:private"],
)

genrule(
    name = "vfkit_kernel_image",
    srcs = ["@alpine_aarch64_vmlinuz_virt//file:vmlinuz-virt"],
    outs = ["vm/_cache/Image"],
    cmd = "$(execpath :extract_kernel_image) --input $(execpath @alpine_aarch64_vmlinuz_virt//file:vmlinuz-virt) --output $@",
    target_compatible_with = [] if _HOST_IS_DARWIN else ["@platforms//:incompatible"],
    tools = [":extract_kernel_image"],
    visibility = ["//visibility:private"],
)

# Pull `binfmt_misc.ko` out of `modloop-virt` (a squashfs). Alpine's
# stock initramfs already has fuse/virtiofs, so binfmt_misc is the only
# module we need to inject. unsquashfs is shelled out from PATH; on
# macOS it lives in /opt/homebrew/bin (brew install squashfs), on Linux
# it's in `squashfs-tools`. If we ever need a hermetic version, we can
# vendor the binary alongside the rest of the vfkit artifacts.
genrule(
    name = "vfkit_binfmt_misc_ko",
    srcs = ["@alpine_aarch64_modloop_virt//file:modloop-virt"],
    outs = ["vm/_cache/binfmt_misc.ko"],
    cmd = " && ".join([
        "export PATH=/opt/homebrew/bin:/opt/homebrew/sbin:/usr/local/bin:$$PATH",
        "if ! command -v unsquashfs >/dev/null; then echo 'unsquashfs not found in PATH (install squashfs-tools or brew install squashfs)'; exit 1; fi",
        "tmp=$$(mktemp -d)",
        "trap \"rm -rf $$tmp\" EXIT",
        "unsquashfs -d $$tmp/sq -no-progress " +
        "$(execpath @alpine_aarch64_modloop_virt//file:modloop-virt) " +
        "modules/" + _ALPINE_KERNEL_VERSION +
        "/kernel/fs/binfmt_misc.ko >/dev/null",
        "cp $$tmp/sq/modules/" + _ALPINE_KERNEL_VERSION +
        "/kernel/fs/binfmt_misc.ko $@",
    ]),
    target_compatible_with = [] if _HOST_IS_DARWIN else ["@platforms//:incompatible"],
    visibility = ["//visibility:private"],
)

genrule(
    name = "vfkit_initramfs",
    srcs = [
        "@alpine_aarch64_initramfs_virt//file:initramfs-virt",
        ":vfkit_binfmt_misc_ko",
        "vm/init",
    ],
    outs = ["vm/_cache/initramfs-aos.cpio.gz"],
    cmd = " ".join([
        "$(execpath :repack_initramfs)",
        "--input $(execpath @alpine_aarch64_initramfs_virt//file:initramfs-virt)",
        "--init $(execpath vm/init)",
        "--kernel-version " + _ALPINE_KERNEL_VERSION,
        "--module fs/binfmt_misc.ko=$(execpath :vfkit_binfmt_misc_ko)",
        "--output $@",
    ]),
    target_compatible_with = [] if _HOST_IS_DARWIN else ["@platforms//:incompatible"],
    tools = [":repack_initramfs"],
    visibility = ["//visibility:private"],
)

filegroup(
    name = "vfkit_cache_files",
    srcs = [
        ":vfkit_initramfs",
        ":vfkit_kernel_image",
    ] if _HOST_IS_DARWIN else [],
    visibility = ["//visibility:private"],
)

filegroup(
    name = "aarch64_files",
    srcs = [
        ":aarch64_synth_root",
        ":vfkit_cache_files",
    ] + ([
        # x86_64 dynamic loader + libc, needed inside the vfkit VM
        # when the Darwin wrapper runs the host tools through Rosetta.
        "@amd64_debian_sysroot//:libc6-filegroup",
    ] if _HOST_IS_DARWIN else []),
)

cuda_toolkit_info(
    name = "aarch64_toolkit_info",
    bin2c = "synth_root/aarch64/bin/bin2c",
    fatbinary = "synth_root/aarch64/bin/fatbinary",
    # All paths resolve under the synthetic root: rules_cuda derives
    # `--cuda-path=` from libdevice's grandparent dir, and clang's CUDA
    # driver looks for bin/ptxas + bin/fatbinary alongside it. The
    # synth root symlinks include/ and nvvm/ back to the real sysroot
    # so headers and libdevice still come from CUDA 12.6.
    libdevice = "synth_root/aarch64/nvvm/libdevice/libdevice.10.bc",
    link_stub = "@rules_cuda//cuda/dummy:link.stub",
    nvlink = "synth_root/aarch64/bin/nvlink",
    path = "",
    ptxas = "synth_root/aarch64/bin/ptxas",
    version = "12.6",
)

cuda_toolchain_config_clang(
    name = "aarch64_clang_config",
    cuda_toolkit = ":aarch64_toolkit_info",
    toolchain_identifier = "clang",
)

cuda_toolchain(
    name = "aarch64_clang",
    compiler_files = [":aarch64_files"],
    compiler_use_cc_toolchain = True,
    toolchain_config = ":aarch64_clang_config",
)

toolchain(
    name = "aarch64_clang_toolchain",
    target_compatible_with = [
        "@platforms//cpu:arm64",
        "//tools/platforms/gpu:nvidia",
    ],
    target_settings = [
        "@rules_cuda//cuda:is_enabled",
        "@rules_cuda//cuda:compiler_is_clang",
    ],
    toolchain = ":aarch64_clang",
    toolchain_type = "@rules_cuda//cuda:toolchain_type",
)

# ---------------------------------------------------------------------------
# No-op toolchain for platforms without GPU (roborio, cortex-m, etc.)
# This allows toolchain resolution to succeed, and then target_compatible_with
# on CUDA targets will skip them during execution.
# ---------------------------------------------------------------------------

# Dummy libdevice for no-op toolchain (rules_cuda doesn't provide this)
exports_files(["dummy_libdevice.bc"])

cuda_toolkit_info(
    name = "no_gpu_toolkit_info",
    bin2c = "@rules_cuda//cuda/dummy:bin2c",
    fatbinary = "@rules_cuda//cuda/dummy:fatbinary",
    libdevice = ":dummy_libdevice.bc",
    link_stub = "@rules_cuda//cuda/dummy:link.stub",
    nvlink = "@rules_cuda//cuda/dummy:nvlink",
    path = "",
    ptxas = "@rules_cuda//cuda/dummy:ptxas",
    version = "0.0",
)

cuda_toolchain_config_clang(
    name = "no_gpu_clang_config",
    cuda_toolkit = ":no_gpu_toolkit_info",
    toolchain_identifier = "clang",
)

cuda_toolchain(
    name = "no_gpu_clang",
    compiler_files = [],
    compiler_use_cc_toolchain = True,
    toolchain_config = ":no_gpu_clang_config",
)

toolchain(
    name = "no_gpu_clang_toolchain",
    target_compatible_with = ["//tools/platforms/gpu:none"],
    target_settings = ["@rules_cuda//cuda:compiler_is_clang"],
    toolchain = ":no_gpu_clang",
    toolchain_type = "@rules_cuda//cuda:toolchain_type",
)
