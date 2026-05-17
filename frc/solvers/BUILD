load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")

cc_library(
    name = "convex",
    hdrs = ["convex.h"],
    visibility = ["//visibility:public"],
    deps = [
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/strings",
        "@eigen",
    ],
)

cc_test(
    name = "convex_test",
    srcs = [
        "convex_test.cc",
    ],
    deps = [
        ":convex",
        "//aos/testing:googletest",
    ],
)

cc_library(
    name = "sparse_convex",
    srcs = ["sparse_convex.cc"],
    hdrs = ["sparse_convex.h"],
    visibility = ["//visibility:public"],
    deps = [
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/strings",
        "@eigen",
    ],
)

cc_test(
    name = "sparse_convex_test",
    srcs = ["sparse_convex_test.cc"],
    deps = [
        ":sparse_convex",
        "//aos/testing:googletest",
    ],
)
