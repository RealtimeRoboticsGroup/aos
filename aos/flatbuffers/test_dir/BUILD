load("@aos//aos/flatbuffers:build_defs.bzl", "flatbuffer_cc_library")
load("@aos//aos/flatbuffers:flatbuffers_python.bzl", "flatbuffer_py_library", "flatbuffer_py_srcs")
load("@rules_shell//shell:sh_binary.bzl", "sh_binary")
load("@rules_shell//shell:sh_test.bzl", "sh_test")
load("//aos/flatbuffers:generate.bzl", "static_flatbuffer")

static_flatbuffer(
    name = "include_reflection_fbs",
    srcs = ["include_reflection.fbs"],
    visibility = ["//visibility:public"],
    deps = ["//aos/flatbuffers/reflection:reflection_fbs"],
)

static_flatbuffer(
    name = "include_fbs",
    srcs = ["include.fbs"],
    visibility = ["//visibility:public"],
)

static_flatbuffer(
    name = "type_coverage_fbs",
    srcs = ["type_coverage.fbs"],
    visibility = ["//visibility:public"],
)

filegroup(
    name = "test_data",
    srcs = ["type_coverage.json"],
    visibility = ["//visibility:public"],
)

sh_binary(
    name = "copy_test_static_header",
    srcs = ["copy_test_static_header.sh"],
    args = ["$(location //aos/flatbuffers:test_static_file)"],
    data = ["//aos/flatbuffers:test_static_file"],
)

sh_test(
    name = "compare_generated_files",
    srcs = ["compare_generated_files.sh"],
    args = [
        "$(location //aos/flatbuffers:test_static_file)",
        "$(location sample_test_static.h)",
    ],
    data = [
        "//aos/flatbuffers:test_static_file",
        "@aos//aos/flatbuffers/test_dir:sample_test_static.h",
    ],
)

flatbuffer_py_srcs(
    name = "external_bar_fbs_py_srcs",
    src = "@mock_external_fbs_repo//:bar.fbs",
)

flatbuffer_py_srcs(
    name = "external_foo_fbs_py_srcs",
    src = "@mock_external_fbs_repo//:foo.fbs",
    deps = [
        ":external_bar_fbs_py_srcs",
    ],
)

flatbuffer_py_library(
    name = "something_fbs_py",
    src = "something.fbs",
    visibility = ["//visibility:public"],
    deps = [
        ":external_foo_fbs_py_srcs",
    ],
)

exports_files(["reflection_test.fbs"])

# Binary Identity Case 1: Local, checked-in fbs
flatbuffer_cc_library(
    name = "reflection_test_fbs",
    srcs = ["reflection_test.fbs"],
    gen_reflections = True,
    visibility = ["//visibility:public"],
)
