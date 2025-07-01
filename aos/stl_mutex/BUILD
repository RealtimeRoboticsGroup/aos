package(
    default_applicable_licenses = ["//:license"],
    default_visibility = ["//visibility:public"],
)

cc_library(
    name = "stl_mutex",
    hdrs = [
        "stl_mutex.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "//aos/ipc_lib:aos_sync",
        "@com_google_absl//absl/log:absl_check",
        "@com_google_absl//absl/log:absl_log",
    ],
)

cc_test(
    name = "stl_mutex_test",
    srcs = [
        "stl_mutex_test.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":stl_mutex",
        "//aos:die",
        "//aos/testing:googletest",
    ],
)
