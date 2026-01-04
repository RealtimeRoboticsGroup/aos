load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")

package(
    default_applicable_licenses = ["//:license"],
    default_visibility = ["//visibility:public"],
)

cc_library(
    name = "stl_mutex",
    hdrs = [
        "stl_mutex.h",
    ],

    deps = [
        "@com_google_absl//absl/log:absl_check",
        "@com_google_absl//absl/log:absl_log",
    ] + select({
        "@platforms//os:linux": ["//aos/ipc_lib:aos_sync"],
        "//conditions:default": [],
    }),
)

cc_test(
    name = "stl_mutex_test",
    srcs = [
        "stl_mutex_test.cc",
    ],

    deps = [
        ":stl_mutex",
        "//aos:die",
        "//aos/testing:googletest",
    ],
)
