load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")

package(
    default_visibility = ["//visibility:public"],
)

cc_library(
    name = "type_traits",
    hdrs = [
        "type_traits.h",
    ],
)

cc_test(
    name = "type_traits_test",
    srcs = [
        "type_traits_test.cpp",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":type_traits",
        "//aos/testing:googletest",
    ],
)
