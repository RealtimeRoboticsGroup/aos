load("//tools/python:defs.bzl", "py_binary")

sh_binary(
    name = "quiet_success",
    srcs = ["quiet_success.sh"],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
)

py_binary(
    name = "jinja2_generator",
    srcs = ["jinja2_generator.py"],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = ["@pip_deps//jinja2"],
)

py_binary(
    name = "apache_runner",
    srcs = ["apache_runner.py"],
    data = [
        "apache_template.conf",
        "@amd64_debian_sysroot//:sysroot_files",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = ["@pip_deps//jinja2"],
)

# Platform-specific libclang for autocxx
alias(
    name = "libclang",
    actual = select({
        "@platforms//os:linux": "@llvm_k8//:libclang",
        "@platforms//os:macos": "@llvm_darwin_aarch64//:libclang",
    }),
    visibility = ["//visibility:public"],
)
