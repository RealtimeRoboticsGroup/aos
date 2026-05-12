load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("//tools/python:defs.bzl", "py_binary")

py_binary(
    name = "gen_embedded",
    srcs = [
        "gen_embedded.py",
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "seasocks_logger",
    srcs = ["seasocks_logger.cc"],
    hdrs = ["seasocks_logger.h"],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "//third_party/seasocks",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
    ],
)
