load("@aos//aos/flatbuffers:typescript.bzl", "flatbuffer_ts_library")
load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")
load("//aos/flatbuffers:generate.bzl", "static_flatbuffer")

static_flatbuffer(
    name = "matrix_fbs",
    srcs = ["matrix.fbs"],
    visibility = ["//visibility:public"],
)

flatbuffer_ts_library(
    name = "matrix_ts_fbs",
    srcs = [
        "matrix.fbs",
    ],
    ts_files = [
        "frc/fbs.ts",
        "frc/fbs/storage-order.ts",
        "frc/fbs/matrix.ts",
        "frc/fbs/matrix-field.ts",
        "frc/fbs/field-error.ts",
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "flatbuffers_matrix",
    hdrs = ["flatbuffers_matrix.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":matrix_fbs",
        "//aos:json_to_flatbuffer",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
        "@eigen",
        "@tl-expected//:tl-expected",
    ],
)

cc_test(
    name = "flatbuffers_matrix_test",
    srcs = ["flatbuffers_matrix_test.cc"],
    deps = [
        ":flatbuffers_matrix",
        "//aos/testing:googletest",
    ],
)

cc_library(
    name = "interpolate",
    hdrs = ["interpolate.h"],
    visibility = ["//visibility:public"],
)

cc_test(
    name = "interpolate_test",
    srcs = ["interpolate_test.cc"],
    deps = [
        ":interpolate",
        "//aos/testing:googletest",
        "@eigen",
    ],
)
