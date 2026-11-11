load("@rules_cc//cc:cc_binary.bzl", "cc_binary")

cc_binary(
    name = "foxglove_ts_generator",
    srcs = ["foxglove_ts_generate.cc"],
    visibility = ["//visibility:public"],
    deps = [
        "//aos:flatbuffers",
        "//aos:init",
        "//aos:json_to_flatbuffer",
        "//aos/util:file",
        "@aos_flatbuffers//:flatbuffers",
        "@com_google_absl//absl/flags:flag",
        "@com_google_absl//absl/log:absl_check",
        "@com_google_absl//absl/log:absl_log",
    ],
)
