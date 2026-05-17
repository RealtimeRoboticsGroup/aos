load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("//aos/flatbuffers:generate.bzl", "static_flatbuffer")
load("//tools/python:defs.bzl", "py_library")

package(default_visibility = ["//visibility:public"])

exports_files(["halide_generator.sh"])

cc_library(
    name = "shifter_hall_effect",
    hdrs = [
        "shifter_hall_effect.h",
    ],
    deps = ["//frc/control_loops/drivetrain:drivetrain_config_fbs"],
)

cc_library(
    name = "constants",
    hdrs = [
        "constants.h",
    ],
    deps = [
        "//frc/control_loops:control_loops_fbs",
        "//frc/zeroing:constants_fbs",
    ],
)

py_library(
    name = "python_init",
    srcs = ["__init__.py"],
    visibility = ["//visibility:public"],
)

static_flatbuffer(
    name = "can_configuration_fbs",
    srcs = [
        ":can_configuration.fbs",
    ],
    visibility = ["//visibility:public"],
)

sh_binary(
    name = "halide_generator_compile_script",
    srcs = [
        "//frc:halide_generator.sh",
    ],
    data = select({
        "@platforms//os:linux": [
            "@amd64_debian_sysroot//:sysroot_files",
            "@halide_k8//:build_files",
            "@llvm_toolchain//:all-components-x86_64-linux",
        ],
        "@platforms//os:macos": [
            "@halide_darwin_aarch64//:build_files",
        ],
        "//conditions:default": [],
    }),
    deps = [
        "@bazel_tools//tools/bash/runfiles",
    ],
)
