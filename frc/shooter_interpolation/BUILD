load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")

cc_library(
    name = "interpolation",
    srcs = [
        "interpolation.cc",
    ],
    hdrs = [
        "interpolation.h",
    ],
    visibility = ["//visibility:public"],
)

cc_test(
    name = "interpolation_test",
    srcs = [
        "interpolation_test.cc",
    ],
    deps = [
        ":interpolation",
        "//aos/testing:googletest",
    ],
)
