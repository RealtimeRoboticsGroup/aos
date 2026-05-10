"""Hermetic CUDA toolchains for rules_cuda.

The CUDA bits we need (`ptxas`, `fatbinary`, libdevice, headers) ship inside
the per-platform sysroot repos, so we just point rules_cuda's clang
toolchain at those file labels. Two toolchains are registered, gated by
`@platforms//cpu`, so Bazel selects the right one for `--platforms`.

Compared to the old approach of stitching symlinks into a hermetic repo,
this keeps every input as a regular Bazel artifact, which is required for
remote execution to work.
"""

load("@rules_cuda//cuda:defs.bzl", "cuda_toolchain", "cuda_toolchain_config_clang", "cuda_toolkit_info")

package(default_visibility = ["//visibility:public"])

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

filegroup(
    name = "aarch64_files",
    srcs = [
        "@arm64_debian_sysroot//:usr/local/cuda-12.6/bin/aarch64-unknown-linux-gnu-fatbinary",
        "@arm64_debian_sysroot//:usr/local/cuda-12.6/bin/bin2c",
        "@arm64_debian_sysroot//:usr/local/cuda-12.6/bin/nvlink",
        "@arm64_debian_sysroot//:usr/local/cuda-12.6/bin/ptxas",
        "@arm64_debian_sysroot//:usr/local/cuda-12.6/nvvm/libdevice/libdevice.10.bc",
    ],
)

cuda_toolkit_info(
    name = "aarch64_toolkit_info",
    bin2c = "@arm64_debian_sysroot//:usr/local/cuda-12.6/bin/bin2c",
    fatbinary = "@arm64_debian_sysroot//:usr/local/cuda-12.6/bin/aarch64-unknown-linux-gnu-fatbinary",
    libdevice = "@arm64_debian_sysroot//:usr/local/cuda-12.6/nvvm/libdevice/libdevice.10.bc",
    link_stub = "@rules_cuda//cuda/dummy:link.stub",
    nvlink = "@arm64_debian_sysroot//:usr/local/cuda-12.6/bin/nvlink",
    path = "",
    ptxas = "@arm64_debian_sysroot//:usr/local/cuda-12.6/bin/ptxas",
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
