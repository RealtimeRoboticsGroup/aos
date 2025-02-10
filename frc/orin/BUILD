load("//aos:flatbuffers.bzl", "cc_static_flatbuffer")
load("//aos/flatbuffers:generate.bzl", "static_flatbuffer")
load("//frc:halide.bzl", "halide_library")

exports_files(["orin_irq_config.json"])

halide_library(
    name = "ycbcr",
    src = "crcv_generator.cc",
    args = "rows=1088 cols=1456 ystride=2048 cbcrstride=3840",
    function = "ycbcr",
    visibility = ["//visibility:public"],
)

halide_library(
    name = "ycbcr422",
    src = "crcv_generator.cc",
    args = "rows=1088 cols=1456 ystride=1536 cbcrstride=1536",
    function = "ycbcr422",
    visibility = ["//visibility:public"],
)

cc_library(
    name = "cuda",
    srcs = [
        "cuda.cc",
    ],
    hdrs = [
        "cuda.h",
    ],
    features = ["cuda"],
    visibility = ["//visibility:public"],
    deps = [
        "//third_party:cudart",
        "@com_github_nvidia_cccl//:cccl",
        "@com_google_absl//absl/flags:flag",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
    ],
)

cc_library(
    name = "apriltag",
    srcs = [
        "apriltag.cc",
        "apriltag_detect.cc",
        "labeling_allegretti_2019_BKE.cc",
        "line_fit_filter.cc",
        "points.cc",
        "threshold.cc",
    ],
    hdrs = [
        "apriltag.h",
        "apriltag_types.h",
        "gpu_image.h",
        "labeling_allegretti_2019_BKE.h",
        "line_fit_filter.h",
        "points.h",
        "threshold.h",
        "transform_output_iterator.h",
    ],
    copts = [
        "-Wno-pass-failed",
        #"-DCUB_DETAIL_DEBUG_ENABLE_LOG=1",
        #"-DDEBUG=1",
    ],
    features = ["cuda"],
    visibility = ["//visibility:public"],
    deps = [
        ":cuda",
        "//aos/time",
        "//frc/vision:vision_fbs",
        "//third_party:cudart",
        "//third_party/apriltag",
        "@com_github_nvidia_cccl//:cccl",
        "@com_google_absl//absl/flags:flag",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
    ],
)

cc_test(
    name = "cuda_april_tag_test",
    srcs = [
        "cuda_april_tag_test.cc",
    ],
    data = [
        "@apriltag_test_bfbs_images",
        "@orin_capture_24_04//file",
        "@orin_capture_24_04_side//file",
        "@orin_image_apriltag//file",
        "@orin_large_image_apriltag//file",
    ],
    features = ["cuda"],
    tags = [
        # This needs a GPU, which means we need the GPU to be accessible inside
        # the container.  Rather than solve that right now, just run it under
        # linux-sandbox on the host which has a GPU already.
        "no-remote-exec",
    ],
    target_compatible_with = ["//tools/platforms/gpu:nvidia"],
    deps = [
        ":apriltag",
        ":ycbcr",
        "//aos:flatbuffer_merge",
        "//aos:json_to_flatbuffer",
        "//aos/testing:googletest",
        "//aos/testing:path",
        "//aos/testing:random_seed",
        "//frc/vision:vision_fbs",
        "//third_party:opencv",
        "//third_party/apriltag",
    ],
)

cc_test(
    name = "output_iterator_test",
    srcs = [
        "output_iterator_test.cc",
    ],
    features = ["cuda"],
    target_compatible_with = ["//tools/platforms/gpu:nvidia"],
    deps = [
        ":apriltag",
        "//aos/testing:googletest",
        "//aos/testing:random_seed",
    ],
)

cc_test(
    name = "points_test",
    srcs = [
        "points_test.cc",
    ],
    features = ["cuda"],
    target_compatible_with = ["//tools/platforms/gpu:nvidia"],
    deps = [
        ":apriltag",
        "//aos/testing:googletest",
        "//aos/testing:random_seed",
    ],
)

cc_binary(
    name = "argus_camera",
    srcs = [
        "argus_camera.cc",
    ],
    target_compatible_with = ["@platforms//cpu:arm64"],
    visibility = ["//visibility:public"],
    deps = [
        "//aos:init",
        "//aos/events:shm_event_loop",
        "//aos/util:file",
        "//frc/orin:ycbcr",
        "//frc/orin:ycbcr422",
        "//frc/vision:vision_fbs",
        "@arm64_debian_sysroot//:argus",
        "@arm64_debian_sysroot//:egl",
        "@arm64_debian_sysroot//:eglstream",
        "@arm64_debian_sysroot//:nvbufsurface",
    ],
)

cc_library(
    name = "gpu_apriltag_lib",
    srcs = ["gpu_apriltag.cc"],
    hdrs = ["gpu_apriltag.h"],
    features = ["cuda"],
    visibility = ["//visibility:public"],
    deps = [
        ":apriltag",
        "//aos:init",
        "//aos/events:shm_event_loop",
        "//frc/constants:constants_sender_lib",
        "//frc/vision:calibration_fbs",
        "//frc/vision:cuda_camera_image_callback",
        "//frc/vision:target_map_fbs",
        "//frc/vision:vision_util_lib",
        "//third_party:opencv",
    ],
)

static_flatbuffer(
    name = "hardware_stats_fbs",
    srcs = ["hardware_stats.fbs"],
    visibility = ["//visibility:public"],
)

cc_static_flatbuffer(
    name = "hardware_stats_schema",
    function = "aos::util::HardwareStatsSchema",
    target = ":hardware_stats_fbs_reflection_out",
)

cc_binary(
    name = "hardware_monitor",
    srcs = ["hardware_monitor.cc"],
    visibility = ["//visibility:public"],
    deps = [
        ":hardware_stats_fbs",
        "//aos:init",
        "//aos/events:shm_event_loop",
        "@com_google_absl//absl/strings",
    ],
)

cc_binary(
    name = "argus_monitor",
    srcs = [
        "argus_monitor.cc",
    ],
    target_compatible_with = ["@platforms//cpu:arm64"],
    visibility = ["//visibility:public"],
    deps = [
        "//aos:aos_cli_utils",
        "//aos:init",
        "//aos/events:shm_event_loop",
    ],
)

cc_binary(
    name = "localizer_logger",
    srcs = [
        "localizer_logger.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "//aos:configuration",
        "//aos:init",
        "//aos/events:shm_event_loop",
        "//aos/events/logging:log_writer",
        "//aos/events/logging:snappy_encoder",
        "//aos/logging:log_namer",
        "@com_google_absl//absl/flags:flag",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
    ],
)

cc_test(
    name = "undistort_test",
    srcs = [
        "undistort_test.cc",
    ],
    data = [
        "//y2024:aos_config",
        "//y2024/constants/calib_files",
    ],
    features = ["cuda"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":apriltag",
        ":gpu_apriltag_lib",
        "//aos:init",
        "//aos/testing:googletest",
        "//aos/testing:path",
        "//frc/vision:calibration_fbs",
        "//frc/vision:charuco_lib",
    ],
)

halide_library(
    name = "ToGreyscaleAndDecimateHalide",
    src = "april_generator.cc",
    args = "rows=720 cols=1280",
    function = "decimate_generator",
    visibility = ["//visibility:public"],
)

halide_library(
    name = "ThresholdHalide",
    src = "april_generator.cc",
    args = "rows=360 cols=640",
    function = "threshold_generator",
    visibility = ["//visibility:public"],
)
