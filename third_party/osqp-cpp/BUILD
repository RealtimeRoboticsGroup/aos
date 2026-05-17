load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")

cc_library(
    name = "osqp-cpp",
    srcs = [
        "src/osqp++.cc",
    ],
    hdrs = [
        "include/osqp++.h",
    ],
    includes = [
        "include",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//third_party/osqp",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@eigen",
    ],
)

cc_test(
    name = "osqp-cpp_test",
    srcs = [
        "test/osqp++_test.cc",
    ],
    deps = [
        ":osqp-cpp",
        "//aos/testing:googletest",
        "@com_google_googletest//:gtest",
    ],
)
