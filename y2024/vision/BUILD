cc_binary(
    name = "apriltag_detector",
    srcs = [
        "apriltag_detector.cc",
        "vision_util.cc",
        "vision_util.h",
    ],
    features = ["cuda"],
    target_compatible_with = ["@platforms//cpu:arm64"],
    visibility = ["//visibility:public"],
    deps = [
        "//aos:configuration",
        "//aos:init",
        "//aos/events:shm_event_loop",
        "//frc/orin:gpu_apriltag_lib",
        "//frc/vision:calibrate_multi_cameras_lib",
        "//third_party:cudart",
        "//third_party/apriltag",
        "//y2024/constants:constants_fbs",
        # "@com_github_gflags_gflags//:gflags",
        # "@com_github_google_glog//:glog",
        "@com_github_nvidia_cccl//:cccl",
        # "@com_github_nvidia_cuco//:cuco",
    ],
)

cc_binary(
    name = "viewer",
    srcs = [
        "viewer.cc",
        "vision_util.cc",
        "vision_util.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = [
        "//y2024:__subpackages__",
    ],
    deps = [
        "//aos:init",
        "//aos:json_to_flatbuffer",
        "//aos/events:shm_event_loop",
        "//frc/constants:constants_sender_lib",
        "//frc/vision:calibrate_multi_cameras_lib",
        "//frc/vision:vision_fbs",
        "//frc/vision:vision_util_lib",
        "//third_party:opencv",
        "//y2024/constants:constants_fbs",
        "@com_google_absl//absl/strings",
    ],
)
