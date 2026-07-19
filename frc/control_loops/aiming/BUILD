load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")

cc_library(
    name = "aiming",
    srcs = ["aiming.cc"],
    hdrs = ["aiming.h"],
    visibility = ["//visibility:public"],
    deps = [
        "//aos/logging",
        "//frc:constants",
        "//frc/control_loops:pose",
        "//frc/zeroing:wrap",
    ],
)

cc_test(
    name = "aiming_test",
    srcs = ["aiming_test.cc"],
    deps = [
        ":aiming",
        "//aos/testing:googletest",
    ],
)
