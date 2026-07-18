load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")

cc_library(
    name = "time",
    srcs = ["time.cc"] + select({
        "@platforms//os:linux": ["time_platform_linux.cc"],
        "@platforms//os:osx": ["time_platform_osx.cc"],
        "@platforms//os:windows": ["time_platform_windows.cc"],
        "//conditions:default": ["time_platform_none.cc"],
    }),
    hdrs = [
        "time.h",
    ],
    visibility = ["//visibility:public"],
    deps = ["//aos:macros"] + select({
        "@platforms//os:linux": [
            "@com_google_absl//absl/log:absl_check",
            "@com_google_absl//absl/strings",
        ],
        "@platforms//os:osx": [
            "@com_google_absl//absl/log:absl_check",
            "@com_google_absl//absl/numeric:int128",
            "@com_google_absl//absl/strings",
        ],
        "@platforms//os:windows": [
            "@com_google_absl//absl/log:absl_check",
            "@com_google_absl//absl/numeric:int128",
            "@com_google_absl//absl/strings",
        ],
        "//conditions:default": ["//motors/core"],
    }),
)

cc_test(
    name = "time_test",
    srcs = [
        "time_test.cc",
    ],
    deps = [
        ":time",
        "//aos/logging",
        "//aos/testing:googletest",
        "//aos/util:death_test_log_implementation",
        "@com_google_absl//absl/log",
        "@google_benchmark//:benchmark",
    ],
)
