load(":defs.bzl", "extract_numpy_headers")

cc_library(
    name = "python",
    defines = [
        "AOS_UPSTREAM_BUNDLED_PYTHON",
    ],
    target_compatible_with = [
        "@platforms//cpu:x86_64",
        "@platforms//os:linux",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "@python3_9_x86_64-unknown-linux-gnu//:libpython",
        "@python3_9_x86_64-unknown-linux-gnu//:python_headers",
    ],
)

filegroup(
    name = "python_runtime",
    data = [
        "@python3_9_x86_64-unknown-linux-gnu//:files",
    ],
    target_compatible_with = [
        "@platforms//cpu:x86_64",
        "@platforms//os:linux",
    ],
    visibility = ["//visibility:public"],
)

extract_numpy_headers(
    name = "numpy_headers",
    header_prefix = "numpy_headers",
    numpy = "@pip//numpy",
    visibility = ["//visibility:private"],
)

cc_library(
    name = "numpy_cc",
    hdrs = [
        ":numpy_headers",
    ],
    includes = [
        "numpy_headers",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":python",
    ],
)
