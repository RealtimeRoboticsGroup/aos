load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")

cc_library(
    name = "time",
    srcs = [
        "time.cc",
    ],
    hdrs = [
        "time.h",
    ],
    visibility = ["//visibility:public"],
    deps = select({
        "@platforms//os:linux": [
            "@com_google_absl//absl/log:absl_check",
            "@com_google_absl//absl/strings",
        ],
        "@platforms//os:osx": [
            "@com_google_absl//absl/log:absl_check",
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
    ],
)
