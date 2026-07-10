load("@bazel_skylib//rules:write_file.bzl", "write_file")
load("@rules_cc//cc:cc_import.bzl", "cc_import")
load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_rust//rust:toolchain.bzl", "rust_stdlib_filegroup", "rust_toolchain")
load("@rules_rust_bindgen//:defs.bzl", "rust_bindgen_toolchain")
load("//tools/build_rules:clean_dep.bzl", "clean_dep")

# We have to declare our toolchains individually to get the corect constraints
# configured so we can robustly select the correct one for each of our
# platforms.

toolchain(
    name = "rust-toolchain-x86",
    exec_compatible_with = [
        "@platforms//os:linux",
        "@platforms//cpu:x86_64",
    ],
    target_compatible_with = [
        "@platforms//os:linux",
        "@platforms//cpu:x86_64",
    ],
    toolchain = "@rust__x86_64-unknown-linux-gnu__stable_tools//:rust_toolchain",
    toolchain_type = "@rules_rust//rust:toolchain",
)

toolchain(
    name = "rust-toolchain-armv7",
    exec_compatible_with = [
        "@platforms//os:linux",
        "@platforms//cpu:x86_64",
    ],
    target_compatible_with = [
        "@platforms//os:linux",
        "@platforms//cpu:armv7",
        # Include this so we're incompatible with the roborio platform, to avoid
        # subtle order dependencies.
        "//tools/platforms/hardware:raspberry_pi",
    ],
    toolchain = "@rust__armv7-unknown-linux-gnueabihf__stable_tools//:rust_toolchain",
    toolchain_type = "@rules_rust//rust:toolchain",
)

toolchain(
    name = "rust-toolchain-arm64",
    exec_compatible_with = [
        "@platforms//os:linux",
        "@platforms//cpu:x86_64",
    ],
    target_compatible_with = [
        "@platforms//os:linux",
        "@platforms//cpu:arm64",
    ],
    toolchain = "@rust__aarch64-unknown-linux-gnu__stable_tools//:rust_toolchain",
    toolchain_type = "@rules_rust//rust:toolchain",
)

toolchain(
    name = "rust-toolchain-roborio",
    exec_compatible_with = [
        "@platforms//os:linux",
        "@platforms//cpu:x86_64",
    ],
    target_compatible_with = [
        "@platforms//os:linux",
        "@platforms//cpu:armv7",
        "//tools/platforms/hardware:roborio",
    ],
    toolchain = "@rust__arm-unknown-linux-gnueabi__stable_tools//:rust_toolchain",
    toolchain_type = "@rules_rust//rust:toolchain",
)

# The remainder of this file exists to create a NOOP toolchain for Rust on
# platforms that don't support Rust. We can probably get rid of this once
# https://github.com/bazelbuild/bazel/issues/12897 is fixed.

write_file(
    name = "noop_error_exit",
    out = "noop_error_exit.sh",
    content = [
        "#!/bin/bash",
        "echo 'This should never be executed. Something went wrong.' >&2",
        "echo 'This NOOP Rust toolchain should never be executed. Something went wrong.' >&2",
        "echo 'Check that your target has `target_compatible_with` set to a platform that supports Rust.' >&2",
        "exit 1",
    ],
    is_executable = True,
)

rust_stdlib_filegroup(
    name = "empty_stdlib",
    srcs = [":noop_error_exit"],
)

rust_toolchain(
    name = "noop_rust_toolchain_impl",
    binary_ext = "",
    cargo = ":noop_error_exit",
    clippy_driver = ":noop_error_exit",
    default_edition = "2021",
    dylib_ext = ".so",
    exec_triple = "none-none-none",
    rust_doc = ":noop_error_exit",
    rust_std = ":empty_stdlib",
    rustc = ":noop_error_exit",
    rustc_lib = ":noop_error_exit",
    rustfmt = ":noop_error_exit",
    staticlib_ext = ".a",
    stdlib_linkflags = [],
    tags = ["manual"],
    target_triple = "none-none-none",
)

toolchain(
    name = "noop_rust_toolchain",
    exec_compatible_with = [
        "@platforms//os:linux",
    ],
    target_compatible_with = [
        clean_dep("//tools/platforms/rust:lacks_support"),
    ],
    toolchain = ":noop_rust_toolchain_impl",
    toolchain_type = "@rules_rust//rust:toolchain",
)

cc_library(
    name = "forward_allocator",
    srcs = ["forward_allocator.c"],
    visibility = ["//visibility:public"],
)

# bindgen parses headers with libclang.  rules_rust_bindgen's default toolchain
# builds clang from @llvm-project source, which is an enormous amount of work to
# repeat for a compiler we already download, so point it at the libclang.so that
# ships in our LLVM distribution instead.
#
# There is no `clang` attribute here on purpose: rust_bindgen only uses that to
# set CLANG_PATH, and the include paths, sysroot and -resource-dir all come from
# the cc_toolchain of whatever is being compiled.  libclang does the parsing.
#
# One of these per host we run bindgen on.  libclang is dlopen'd by the bindgen
# binary itself, so it has to come from the execution platform's LLVM
# distribution rather than the target's.

cc_import(
    name = "libclang",
    shared_library = "@llvm_k8//:lib/libclang.so",
    tags = ["manual"],
)

cc_import(
    name = "libclang_darwin",
    shared_library = "@llvm_darwin_aarch64//:lib/libclang.dylib",
    tags = ["manual"],
)

rust_bindgen_toolchain(
    name = "bindgen_toolchain_impl",
    bindgen = "@rules_rust_bindgen//3rdparty:bindgen",
    libclang = ":libclang",
    tags = ["manual"],
)

rust_bindgen_toolchain(
    name = "bindgen_toolchain_darwin_impl",
    bindgen = "@rules_rust_bindgen//3rdparty:bindgen",
    libclang = ":libclang_darwin",
    tags = ["manual"],
)

toolchain(
    name = "bindgen_toolchain",
    exec_compatible_with = [
        "@platforms//os:linux",
        "@platforms//cpu:x86_64",
    ],
    toolchain = ":bindgen_toolchain_impl",
    toolchain_type = "@rules_rust_bindgen//:toolchain_type",
)

toolchain(
    name = "bindgen_toolchain_darwin",
    exec_compatible_with = [
        "@platforms//os:macos",
        "@platforms//cpu:arm64",
    ],
    toolchain = ":bindgen_toolchain_darwin_impl",
    toolchain_type = "@rules_rust_bindgen//:toolchain_type",
)
