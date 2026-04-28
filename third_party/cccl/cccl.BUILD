cc_library(
    name = "cccl",
    hdrs = glob(include = [
        "thrust/thrust/**",
        "libcudacxx/include/**",
        "cub/cub/**",
    ]),
    includes = [
        "cub",
        "libcudacxx/include",
        "thrust",
    ],
    target_compatible_with = [
        "@aos//tools/platforms/gpu:nvidia",
        "@platforms//os:linux",
    ],
    visibility = ["//visibility:public"],
)
