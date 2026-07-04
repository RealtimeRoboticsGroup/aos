load("@aspect_rules_lint//format:defs.bzl", "format_multirun")
load("@ci_configure//:ci.bzl", "RUNNING_IN_CI")
load("@npm//:prettier/package_json.bzl", prettier_bin = "bin")
load("@rules_multirun//:defs.bzl", "multirun")
load("@rules_python//python/entry_points:py_console_script_binary.bzl", "py_console_script_binary")

prettier_bin.prettier_binary(
    name = "prettier_binary",
)

sh_binary(
    name = "prettier_wrapped",
    srcs = ["prettier_wrapper.sh"],
    data = [":prettier_binary"],
    env = {
        "BAZEL_BINDIR": ".",
    },
    deps = [
        "@bazel_tools//tools/bash/runfiles",
    ],
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
    name = "go_mod_tidy",
    srcs = ["go_mod_tidy.sh"],
    data = ["@go_sdk//:bin/go"],
    env = {
        "RUNNING_IN_CI": "1" if RUNNING_IN_CI else "0",
    },
    deps = [
        "@bazel_tools//tools/bash/runfiles",
    ],
)

sh_binary(
    name = "update_go_repos",
    srcs = ["update_go_repos.sh"],
    data = ["//:gazelle"],
    deps = [
        "@bazel_tools//tools/bash/runfiles",
    ],
)

sh_binary(
    name = "clean_up_go_mirrors",
    srcs = ["clean_up_go_mirrors.sh"],
    data = ["//tools/go:mirror_go_repos"],
    deps = [
        "@bazel_tools//tools/bash/runfiles",
    ],
)

sh_binary(
    name = "tweak_gazelle_go_deps",
    srcs = ["tweak_gazelle_go_deps.sh"],
    data = ["//tools/go:tweak_gazelle_go_deps"],
    deps = [
        "@bazel_tools//tools/bash/runfiles",
    ],
)

multirun(
    name = "run-ci",
    commands = [
        ":format.check",
        ":go_mod_tidy",
        ":update_go_repos",
        ":tweak_gazelle_go_deps",
        ":clean_up_go_mirrors",
    ],
    target_compatible_with = ["//tools/platforms/go:has_support"],
    visibility = ["//visibility:public"],
)

alias(
    name = "clang_format_binary",
    actual = select({
        "//tools:arm64_linux": "@llvm_aarch64//:bin/clang-format",
        "//tools:arm64_macos": "@llvm_darwin_aarch64//:bin/clang-format",
        "//conditions:default": "@llvm_k8//:bin/clang-format",
    }),
)

format_multirun(
    name = "format",
    cc = ":clang_format_binary",
    css = ":prettier_wrapped",
    go = "@go_sdk//:bin/gofmt",
    html = ":prettier_wrapped",
    javascript = ":prettier_wrapped",
    markdown = ":prettier_wrapped",
    python = ":yapf_binary",
    python_check_args = [
        "--diff",
        "--parallel",
    ],
    python_fix_args = [
        "--in-place",
        "--parallel",
    ],
    rust = "@aos_rust_host_tools//:rustfmt",
    rust_check_args = [
        "--edition=2021",
        "--check",
    ],
    rust_fix_args = ["--edition=2021"],
    starlark = "@buildifier_prebuilt//:buildifier",
    starlark_check_args = [
        "-mode=check",
        "-warnings=all",
        "--lint=warn",
        "--warnings=-module-docstring,-function-docstring,-function-docstring-args,-rule-impl-return,-no-effect,-provider-params,-unnamed-macro,-positional-args",
    ],
    starlark_fix_args = [
        "-mode=fix",
        "-warnings=all",
        "--lint=fix",
    ],
    visibility = ["//visibility:public"],
)
