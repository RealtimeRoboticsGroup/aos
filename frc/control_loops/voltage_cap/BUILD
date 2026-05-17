load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "voltage_cap",
    srcs = [
        "voltage_cap.cc",
    ],
    hdrs = [
        "voltage_cap.h",
    ],
)

cc_test(
    name = "voltage_cap_test",
    srcs = [
        "voltage_cap_test.cc",
    ],
    deps = [
        ":voltage_cap",
        "//aos/testing:googletest",
    ],
)
