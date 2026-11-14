load("@rules_cc//cc:cc_binary.bzl", "cc_binary")
load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")
load("//tools:cuda_library.bzl", "cuda_library")

cc_library(
    name = "target_mapper",
    srcs = ["target_mapper.cc"],
    hdrs = ["target_mapper.h"],
    data = ["target_map.json"],
    visibility = ["//visibility:public"],
    deps = [
        "//aos/events:simulated_event_loop",
        "//frc/control_loops:control_loop",
        "//frc/vision:target_map_fbs",
        "//frc/vision/ceres:pose_graph_3d_lib",
        "//frc/vision/util:geometry_lib",
        "//frc/vision/util:vision_util_lib",
        "//frc/vision/util:visualize_robot",
        "//third_party:opencv",
        "@ceres-solver//:ceres",
        "@com_google_absl//absl/flags:flag",
        "@com_google_absl//absl/log:absl_log",
        "@eigen",
    ],
)

cc_test(
    name = "target_mapper_test",
    srcs = [
        "target_mapper_test.cc",
    ],
    data = [":target_map.json"],
    deps = [
        ":target_mapper",
        "//aos/events:simulated_event_loop",
        "//aos/testing:googletest",
        "//aos/testing:path",
        "//aos/testing:random_seed",
        "//aos/util:math",
    ],
)

cc_library(
    name = "target_map_utils",
    srcs = ["target_map_utils.cc"],
    hdrs = ["target_map_utils.h"],
    visibility = ["//visibility:public"],
    deps = [
        "//frc/vision:target_map_fbs",
        "@eigen",
    ],
)

cuda_library(
    name = "apriltag_detector_lib",
    srcs = ["apriltag_detector.cc"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "//aos:configuration",
        "//aos:init",
        "//aos/events:shm_event_loop",
        "//frc/orin:gpu_apriltag_lib",
        "//frc/vision:camera_constants_fbs",
        "//third_party:cudart",
        "//third_party/apriltag",
        "@com_github_nvidia_cccl//:cccl",
        "@com_google_absl//absl/flags:flag",
        "@com_google_absl//absl/log:absl_check",
        "@com_google_absl//absl/log:absl_log",
    ],
    alwayslink = True,
)

cc_binary(
    name = "apriltag_detector",
    target_compatible_with = ["//tools/platforms/gpu:nvidia"],
    visibility = ["//visibility:public"],
    deps = [":apriltag_detector_lib"],
)

cc_binary(
    name = "yolo",
    srcs = [
        "yolo.cc",
    ],
    target_compatible_with = ["//tools/platforms/gpu:nvidia"],
    visibility = ["//visibility:public"],
    deps = [
        "//aos:configuration",
        "//aos:init",
        "//aos/events:shm_event_loop",
        "//aos/util:file",
        "//frc/orin:cuda",
        "//frc/orin:resize_normalize",
        "//frc/vision:coral_detection_fbs",
        "//frc/vision:vision_fbs",
        "//frc/vision/camera:cuda_camera_image_callback",
        "//third_party:cudart",
        "//third_party:opencv",
        "//third_party:tensorrt",
        "@com_google_absl//absl/flags:flag",
        "@com_google_absl//absl/log:absl_check",
        "@com_google_absl//absl/log:absl_log",
    ],
)
