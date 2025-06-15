load("@com_github_google_flatbuffers//:typescript.bzl", "flatbuffer_ts_library")

flatbuffer_ts_library(
    name = "include_ts_fbs",
    srcs = ["include.fbs"],
    include_reflection = True,
    target_compatible_with = ["@platforms//cpu:x86_64"],
    ts_files = [
        "aos/examples2/parent.ts",
        "aos/examples/foo.ts",
    ],
    deps = ["//build_tests:test_ts_fbs"],
)
