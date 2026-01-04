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
    target_compatible_with = ["@platforms//os:linux"],
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
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":aos_strsignal",
        "//aos:sanitizers",
        "//aos/testing:googletest",
    ],
)

cc_library(
    name = "dirname",
    srcs = [
        "dirname.cc",
    ],
    hdrs = [
        "dirname.h",
    ],
)

cc_test(
    name = "dirname_test",
    srcs = [
        "dirname_test.cc",
    ],
    deps = [
        ":dirname",
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
