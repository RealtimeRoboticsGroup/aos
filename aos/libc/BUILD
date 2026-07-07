load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")

package(
    default_applicable_licenses = ["//:license"],
    default_visibility = ["//visibility:public"],
)

cc_library(
    name = "aos_strsignal",
    srcs = [
        "aos_strsignal.cc",
    ],
    hdrs = [
        "aos_strsignal.h",
    ],
    deps = [
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
    ],
)

cc_test(
    name = "aos_strsignal_test",
    srcs = [
        "aos_strsignal_test.cc",
    ],
    target_compatible_with = select({
        "@platforms//os:windows": ["@platforms//:incompatible"],
        "//conditions:default": [],
    }),
    deps = [
        ":aos_strsignal",
        "//aos:sanitizers",
        "//aos/testing:googletest",
    ],
)

cc_library(
    name = "aos_strerror",
    srcs = [
        "aos_strerror.cc",
    ],
    hdrs = [
        "aos_strerror.h",
    ],
)

cc_test(
    name = "aos_strerror_test",
    srcs = [
        "aos_strerror_test.cc",
    ],
    deps = [
        ":aos_strerror",
        "//aos/testing:googletest",
    ],
)
