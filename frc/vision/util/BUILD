load("@rules_cc//cc:cc_binary.bzl", "cc_binary")
load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")

cc_library(
    name = "charuco_lib",
    srcs = [
        "charuco_lib.cc",
    ],
    hdrs = [
        "charuco_lib.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "//aos:flatbuffers",
        "//aos/events:event_loop",
        "//aos/network:message_bridge_server_fbs",
        "//aos/network:team_number",
        "//frc/control_loops:quaternion_utils",
        "//frc/vision:calibration_fbs",
        "//frc/vision:vision_fbs",
        "//third_party:opencv",
        "@com_github_foxglove_schemas//:schemas",
        "@com_google_absl//absl/log:absl_check",
        "@com_google_absl//absl/log:absl_log",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/types:span",
        "@eigen",
    ],
)

cc_library(
    name = "geometry_lib",
    hdrs = [
        "geometry.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//aos/util:math",
        "//third_party:opencv",
        "@com_google_absl//absl/log:absl_check",
        "@com_google_absl//absl/log:absl_log",
    ],
)

cc_test(
    name = "geometry_test",
    srcs = [
        "geometry_test.cc",
    ],
    deps = [
        ":geometry_lib",
        "//aos/testing:googletest",
    ],
)

cc_library(
    name = "visualize_robot",
    srcs = [
        "visualize_robot.cc",
    ],
    hdrs = [
        "visualize_robot.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//aos:init",
        "//third_party:opencv",
        "@com_google_absl//absl/log:absl_check",
        "@com_google_absl//absl/log:absl_log",
        "@com_google_absl//absl/strings:str_format",
        "@eigen",
    ],
)

cc_binary(
    name = "visualize_robot_sample",
    srcs = [
        "visualize_robot_sample.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":visualize_robot",
        "//aos:init",
        "//third_party:opencv",
        "@ceres-solver//:ceres",
        "@com_google_absl//absl/log:absl_check",
        "@com_google_absl//absl/log:absl_log",
        "@eigen",
    ],
)

cc_library(
    name = "foxglove_image_converter_lib",
    srcs = ["foxglove_image_converter_lib.cc"],
    hdrs = ["foxglove_image_converter_lib.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":charuco_lib",
        "//aos/events:event_loop",
        "//frc/vision:vision_fbs",
        "//third_party:opencv",
        "@com_github_foxglove_schemas//:schemas",
        "@com_google_absl//absl/log:absl_check",
        "@com_google_absl//absl/log:absl_log",
    ],
)

cc_library(
    name = "vision_util_lib",
    srcs = ["vision_util_lib.cc"],
    hdrs = ["vision_util_lib.h"],
    visibility = ["//visibility:public"],
    deps = [
        "//aos/util:math",
        "//frc/control_loops:quaternion_utils",
        "//frc/vision:calibration_fbs",
        "//frc/vision/ceres:pose_graph_3d_lib",
        "//third_party:opencv",
        "@ceres-solver//:ceres",
        "@com_google_absl//absl/log:absl_check",
        "@com_google_absl//absl/log:absl_log",
        "@com_google_absl//absl/strings:str_format",
        "@eigen",
    ],
)

cc_test(
    name = "vision_util_lib_test",
    srcs = ["vision_util_lib_test.cc"],
    visibility = ["//visibility:public"],
    deps = [
        ":vision_util_lib",
        "//aos/testing:googletest",
        "@com_google_absl//absl/log:absl_check",
        "@com_google_absl//absl/log:absl_log",
    ],
)
