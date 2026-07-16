load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")

cc_library(
    name = "trajectory",
    srcs = [
        "trajectory.cc",
    ],
    hdrs = [
        "trajectory.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":dynamics",
        "//aos/logging",
        "//frc/control_loops:dlqr",
        "//frc/control_loops:jacobian",
        "@eigen",
    ],
)

cc_test(
    name = "trajectory_test",
    srcs = [
        "trajectory_test.cc",
    ],
    deps = [
        ":demo_path",
        ":dynamics",
        ":ekf",
        ":test_constants",
        ":trajectory",
        "//aos/testing:googletest",
        "@eigen",
    ],
)

cc_library(
    name = "dynamics",
    srcs = [
        "dynamics.cc",
    ],
    hdrs = [
        "dynamics.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//frc/control_loops:runge_kutta",
        "@com_google_absl//absl/flags:flag",
        "@eigen",
    ],
)

cc_library(
    name = "demo_path",
    srcs = [
        "demo_path.cc",
    ],
    hdrs = ["demo_path.h"],
    visibility = ["//visibility:public"],
    deps = [":trajectory"],
)

cc_test(
    name = "dynamics_test",
    srcs = [
        "dynamics_test.cc",
    ],
    deps = [
        ":dynamics",
        ":test_constants",
        "//aos/testing:googletest",
    ],
)

cc_library(
    name = "ekf",
    srcs = [
        "ekf.cc",
    ],
    hdrs = [
        "ekf.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":dynamics",
        "//frc/control_loops:jacobian",
        "@eigen",
    ],
)

cc_library(
    name = "graph",
    srcs = ["graph.cc"],
    hdrs = ["graph.h"],
    visibility = ["//visibility:public"],
)

cc_test(
    name = "graph_test",
    srcs = ["graph_test.cc"],
    deps = [
        ":graph",
        "//aos/testing:googletest",
    ],
)

cc_library(
    name = "test_constants",
    hdrs = ["test_constants.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":dynamics",
    ],
)
