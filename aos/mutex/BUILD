load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")

package(
    default_applicable_licenses = ["//:license"],
    default_visibility = ["//visibility:public"],
)

cc_library(
    name = "mutex",
    srcs = [
        "mutex.cc",
    ],
    hdrs = [
        "mutex.h",
    ],
    deps = [
        "//aos/ipc_lib:aos_sync",
        "//aos/type_traits",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
    ],
)

cc_test(
    name = "mutex_test",
    srcs = [
        "mutex_test.cc",
    ],
    deps = [
        ":mutex",
        "//aos/testing:googletest",
        "//aos/testing:test_logging",
        "//aos/testing:test_shm",
        "//aos/time",
        "//aos/util:death_test_log_implementation",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
    ],
)
