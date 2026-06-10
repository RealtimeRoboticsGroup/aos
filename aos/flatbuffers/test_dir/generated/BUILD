load("@aos//aos/flatbuffers:build_defs.bzl", "flatbuffer_cc_library")

# Binary Identity Case 2: Local, generated fbs
genrule(
    name = "generate_reflection_test_fbs",
    srcs = ["//aos/flatbuffers/test_dir:reflection_test.fbs"],
    outs = ["reflection_test.fbs"],
    cmd = "cp $< $@",
)

flatbuffer_cc_library(
    name = "reflection_test_fbs",
    srcs = [":reflection_test.fbs"],
    gen_reflections = True,
    visibility = ["//visibility:public"],
)
