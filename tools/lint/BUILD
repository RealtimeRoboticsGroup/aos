load("@ci_configure//:ci.bzl", "RUNNING_IN_CI")
load("@npm//:prettier/package_json.bzl", prettier_bin = "bin")
load("@rules_python//python/entry_points:py_console_script_binary.bzl", "py_console_script_binary")

prettier_bin.prettier_binary(
    name = "prettier_binary",
)

sh_binary(
    name = "clang_format",
    srcs = ["clang_format.sh"],
    data = select({
        "//tools:arm64_linux": ["@llvm_aarch64//:bin"],
        "//tools:arm64_macos": ["@llvm_darwin_aarch64//:bin"],
        "//conditions:default": ["@llvm_k8//:bin"],
    }),
    deps = [
        "@bazel_tools//tools/bash/runfiles",
    ],
)

sh_binary(
    name = "gofmt",
    srcs = ["gofmt.sh"],
    data = [
        "@go_sdk//:bin/gofmt",
    ],
    target_compatible_with = ["//tools/platforms/go:has_support"],
    deps = [
        "@bazel_tools//tools/bash/runfiles",
    ],
)

sh_binary(
    name = "rustfmt",
    srcs = ["rustfmt.sh"],
    data = [
        "@aos_rust_host_tools//:rustfmt",
    ],
    deps = [
        "@bazel_tools//tools/bash/runfiles",
    ],
)

sh_binary(
    name = "buildifier",
    srcs = ["buildifier.sh"],
    data = [
        "@buildifier_prebuilt//:buildifier",
    ],
    deps = [
        "@bazel_tools//tools/bash/runfiles",
    ],
)

py_console_script_binary(
    name = "yapf_binary",
    pkg = "@pip_deps//yapf",
    script = "yapf",
)

sh_binary(
    name = "yapf",
    srcs = ["yapf.sh"],
    data = [
        ":yapf_binary",
    ],
    deps = [
        "@bazel_tools//tools/bash/runfiles",
    ],
)

sh_binary(
    name = "prettier",
    srcs = ["prettier.sh"],
    data = [
        ":prettier_binary",
    ],
    deps = [
        "@bazel_tools//tools/bash/runfiles",
    ],
)

sh_binary(
    name = "run-ci",
    srcs = [
        "run-ci.sh",
    ],
    data = [
        ":buildifier",
        ":clang_format",
        ":gofmt",
        ":prettier",
        ":rustfmt",
        ":yapf",
        "//:gazelle-runner",
        "//tools/go:mirror_go_repos",
        "//tools/go:tweak_gazelle_go_deps",
        "@go_sdk//:bin/go",
    ],
    env = {
        # Prevent CI errors like:
        #   failed to initialize build cache at
        #   /var/lib/buildkite-agent/.cache/go-build: permission denied
        "RUNNING_IN_CI": "1" if RUNNING_IN_CI else "0",
    },
    target_compatible_with = ["//tools/platforms/go:has_support"],
    deps = [
        "@bazel_tools//tools/bash/runfiles",
    ],
)
