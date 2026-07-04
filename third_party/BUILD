# Incompatible library in case one isn't available for a specific architecture.

load("@rules_cc//cc:cc_library.bzl", "cc_library")

cc_library(
    name = "unavailable",
    target_compatible_with = ["@platforms//:incompatible"],
)

cc_library(
    name = "wpilib",
    linkstatic = True,
    target_compatible_with = ["//tools/platforms/hardware:roborio"],
    visibility = ["//visibility:public"],
    deps = ["//frc/wpilib/ahal"],
)

cc_library(
    name = "wpilib_hal",
    target_compatible_with = ["//tools/platforms/hardware:roborio"],
    visibility = ["//visibility:public"],
    deps = ["@com_github_wpilibsuite_allwpilib//hal:wpiHal.static"],
)

cc_library(
    name = "phoenix",
    linkopts = [
        "-Wl,-rpath",
        "-Wl,.",
    ],
    target_compatible_with = ["//tools/platforms/hardware:roborio"],
    visibility = ["//visibility:public"],
    deps = [
        "@ctre_phoenix6_tools_athena//:tools",
        "@ctre_phoenix6_tools_headers//:tools",
        "@ctre_phoenix_api_cpp_athena//:api-cpp",
        "@ctre_phoenix_api_cpp_headers//:api-cpp",
        "@ctre_phoenix_cci_athena//:cci",
        "@ctre_phoenix_cci_headers//:cci",
    ],
)

cc_library(
    name = "phoenix6",
    linkopts = [
        "-Wl,-rpath",
        "-Wl,.",
    ],
    visibility = ["//visibility:public"],
    deps = select({
        "//tools/platforms/hardware:roborio": [
            "//third_party:wpilib",
            "@ctre_phoenix6_api_cpp_athena//:api-cpp",
            "@ctre_phoenix6_api_cpp_headers//:api-cpp",
            "@ctre_phoenix6_tools_athena//:tools",
            "@ctre_phoenix6_tools_headers//:tools",
        ],
        "@platforms//cpu:arm64": [
            "@ctre_phoenix6_arm64//:headers",
            "@ctre_phoenix6_arm64//:shared_libraries_lib",
        ],
        "//conditions:default": [":unavailable"],
    }),
)

cc_library(
    name = "opencv",
    target_compatible_with = select({
        "//tools:has_msan": ["@platforms//:incompatible"],
        "//conditions:default": [],
    }),
    visibility = ["//visibility:public"],
    deps = select({
        "//tools:arm64_macos": [":unavailable"],
        "//tools:cpu_arm64": ["@arm64_debian_sysroot//:opencv4"],
        "//tools:cpu_k8": ["@amd64_debian_sysroot//:opencv4"],
        "//conditions:default": [":unavailable"],
    }),
)

cc_library(
    name = "gstreamer",
    target_compatible_with = select({
        "//tools:has_msan": ["@platforms//:incompatible"],
        "//conditions:default": [],
    }),
    visibility = ["//visibility:public"],
    deps = select({
        "//tools:arm64_macos": [":unavailable"],
        "//tools:cpu_arm64": [
            "@arm64_debian_sysroot//:gstreamer-1.0",
            "@arm64_debian_sysroot//:gstreamer-app-1.0",
            "@arm64_debian_sysroot//:gstreamer-webrtc-1.0",
        ],
        "//tools:cpu_k8": [
            "@amd64_debian_sysroot//:gstreamer-1.0",
            "@amd64_debian_sysroot//:gstreamer-app-1.0",
            "@amd64_debian_sysroot//:gstreamer-webrtc-1.0",
        ],
        "//conditions:default": [":unavailable"],
    }),
)

cc_library(
    name = "cudart",
    target_compatible_with = select({
        "//tools:has_msan": ["@platforms//:incompatible"],
        "//conditions:default": [],
    }),
    visibility = ["//visibility:public"],
    deps = select({
        "//tools:arm64_macos": [":unavailable"],
        "//tools:cpu_arm64": ["@arm64_debian_sysroot//:cudart-12.6"],
        "//tools:cpu_k8": ["@amd64_debian_sysroot//:cudart"],
        "//conditions:default": [":unavailable"],
    }),
)

cc_library(
    name = "tensorrt",
    target_compatible_with = select({
        "//tools:has_msan": ["@platforms//:incompatible"],
        "//conditions:default": [],
    }),
    visibility = ["//visibility:public"],
    deps = select({
        "//tools:arm64_macos": [":unavailable"],
        "//tools:cpu_arm64": ["@arm64_debian_sysroot//:tensorrt"],
        "//tools:cpu_k8": ["@amd64_tensorrt//:tensorrt"],
        "//conditions:default": [":unavailable"],
    }),
)

cc_library(
    name = "libturbojpeg",
    target_compatible_with = select({
        "//tools:has_msan": ["@platforms//:incompatible"],
        "//conditions:default": [],
    }),
    visibility = ["//visibility:public"],
    deps = select({
        "//tools:arm64_macos": [":unavailable"],
        "//tools:cpu_arm64": ["@arm64_debian_sysroot//:libturbojpeg"],
        "//tools:cpu_k8": ["@amd64_debian_sysroot//:libturbojpeg"],
        "//conditions:default": [":unavailable"],
    }),
)
