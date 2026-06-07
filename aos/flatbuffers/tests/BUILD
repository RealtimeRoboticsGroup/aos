load("@aos//aos/flatbuffers:flatbuffers_python.bzl", "flatbuffer_py_library")

flatbuffer_py_library(
    name = "bar_fbs_py",
    src = "bar.fbs",
    visibility = ["//visibility:public"],
)

flatbuffer_py_library(
    name = "foo_fbs_py",
    src = "foo.fbs",
    visibility = ["//visibility:public"],
    deps = [
        ":bar_fbs_py_srcs",
    ],
)
