load("@com_github_google_flatbuffers//:build_defs.bzl", "flatbuffer_py_library")
load("@com_github_google_flatbuffers//:typescript.bzl", "flatbuffer_ts_library")
load("//aos:config.bzl", "aos_config")
load("//aos/flatbuffers:generate.bzl", "static_flatbuffer")
load("//frc:downloader.bzl", "robot_downloader")
load("//tools/build_rules:template.bzl", "jinja2_template")

cc_binary(
    name = "modify_extrinsics",
    srcs = [
        "modify_extrinsics.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":calibration_fbs",
        "//aos:configuration",
        "//aos:init",
        "//aos/events:event_loop",
        "//frc/vision:vision_util_lib",
        "@com_google_absl//absl/strings:str_format",
        "@org_tuxfamily_eigen//:eigen",
    ],
)

cc_binary(
    name = "image_replay",
    srcs = [
        "image_replay.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":vision_fbs",
        "//aos:configuration",
        "//aos:init",
        "//aos/events:simulated_event_loop",
        "//aos/events/logging:log_reader",
        "//third_party:opencv",
    ],
)

cc_binary(
    name = "image_logger",
    srcs = [
        "image_logger.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":vision_fbs",
        "//aos:configuration",
        "//aos:init",
        "//aos/events:shm_event_loop",
        "//aos/events/logging:log_writer",
        "//aos/logging:log_namer",
        "//aos/util:filesystem_fbs",
        "//frc/input:joystick_state_fbs",
    ],
)

cc_binary(
    name = "foxglove_image_converter",
    srcs = ["foxglove_image_converter.cc"],
    visibility = ["//visibility:public"],
    deps = [
        "//aos:init",
        "//aos/events:shm_event_loop",
        "//frc/vision:foxglove_image_converter_lib",
    ],
)

static_flatbuffer(
    name = "vision_fbs",
    srcs = ["vision.fbs"],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
)

flatbuffer_ts_library(
    name = "vision_ts_fbs",
    srcs = ["vision.fbs"],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
)

static_flatbuffer(
    name = "calibration_fbs",
    srcs = ["calibration.fbs"],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
)

flatbuffer_ts_library(
    name = "calibration_ts_fbs",
    srcs = ["calibration.fbs"],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
)

flatbuffer_ts_library(
    name = "target_map_ts_fbs",
    srcs = ["target_map.fbs"],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
)

flatbuffer_py_library(
    name = "calibration_fbs_python",
    srcs = [
        "calibration.fbs",
    ],
    namespace = "frc.vision.calibration",
    tables = [
        "CalibrationData",
        "CameraCalibration",
        "TransformationMatrix",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
)

py_library(
    name = "create_calib_file",
    srcs = [
        "create_calib_file.py",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":calibration_fbs_python",
        "@bazel_tools//tools/python/runfiles",
        "@pip//glog",
        "@pip//opencv_python",
    ],
)

cc_library(
    name = "v4l2_reader",
    srcs = [
        "v4l2_reader.cc",
    ],
    hdrs = [
        "v4l2_reader.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":vision_fbs",
        "//aos/events:epoll",
        "//aos/events:event_loop",
        "//aos/scoped:scoped_fd",
        "//aos/util:threaded_consumer",
        "@com_google_absl//absl/base",
    ],
)

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
        ":vision_fbs",
        "//aos:flatbuffers",
        "//aos/events:event_loop",
        "//aos/network:message_bridge_server_fbs",
        "//aos/network:team_number",
        "//frc/control_loops:quaternion_utils",
        "//frc/vision:calibration_fbs",
        "//third_party:opencv",
        "@com_github_foxglove_schemas//:schemas",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/types:span",
        "@org_tuxfamily_eigen//:eigen",
    ],
)

cc_library(
    name = "extrinsics_calibration",
    srcs = [
        "calibration_accumulator.cc",
        "calibration_accumulator.h",
        "extrinsics_calibration.cc",
        "extrinsics_calibration.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":charuco_lib",
        ":foxglove_image_converter_lib",
        "//aos:init",
        "//aos/events/logging:log_reader",
        "//frc/control_loops/drivetrain:improved_down_estimator",
        "//frc/vision:visualize_robot",
        "//frc/wpilib:imu_batch_fbs",
        "//frc/wpilib:imu_fbs",
        "//third_party:opencv",
        "@com_github_foxglove_schemas//:CompressedImage_schema",
        "@com_github_foxglove_schemas//:ImageAnnotations_schema",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_ceres_solver//:ceres",
        "@org_tuxfamily_eigen//:eigen",
    ] + [
        # TODO(Stephan):  This is a whacky hack.  If we include this
        # in the proper spot above (alphabetically), then we get a
        # linker error: duplicate symbol: crc32.
        # It's part of both @zlib and @com_github_rawrtc_re.
        "//aos/analysis:in_process_plotter",
    ],
)

static_flatbuffer(
    name = "target_map_fbs",
    srcs = ["target_map.fbs"],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "target_mapper",
    srcs = ["target_mapper.cc"],
    hdrs = ["target_mapper.h"],
    data = ["target_map.json"],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":geometry_lib",
        ":target_map_fbs",
        "//aos/events:simulated_event_loop",
        "//frc/control_loops:control_loop",
        "//frc/vision:vision_util_lib",
        "//frc/vision:visualize_robot",
        "//frc/vision/ceres:pose_graph_3d_lib",
        "//third_party:opencv",
        "@com_google_ceres_solver//:ceres",
        "@org_tuxfamily_eigen//:eigen",
    ],
)

cc_test(
    name = "target_mapper_test",
    srcs = [
        "target_mapper_test.cc",
    ],
    data = [":target_map.json"],
    target_compatible_with = ["@platforms//os:linux"],
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
    name = "geometry_lib",
    hdrs = [
        "geometry.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "//aos/util:math",
        "//third_party:opencv",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
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
        "@com_google_absl//absl/strings:str_format",
        "@org_tuxfamily_eigen//:eigen",
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
        "//aos:init",
        "//frc/vision:visualize_robot",
        "//third_party:opencv",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
        "@com_google_ceres_solver//:ceres",
        "@org_tuxfamily_eigen//:eigen",
    ],
)

cc_library(
    name = "media_device",
    srcs = [
        "media_device.cc",
    ],
    hdrs = ["media_device.h"],
    visibility = ["//visibility:public"],
    deps = [
        "//aos/scoped:scoped_fd",
        "//aos/util:file",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/strings",
    ],
)

cc_library(
    name = "foxglove_image_converter_lib",
    srcs = ["foxglove_image_converter_lib.cc"],
    hdrs = ["foxglove_image_converter_lib.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":charuco_lib",
        ":vision_fbs",
        "//aos/events:event_loop",
        "//third_party:opencv",
        "@com_github_foxglove_schemas//:schemas",
    ],
)

aos_config(
    name = "aos_config",
    src = "single_node_camera_config.json",
    flatbuffers = [
        ":vision_fbs",
        ":target_map_fbs",
        ":camera_constants_fbs",
        "//aos/events:event_loop_fbs",
        "//aos/logging:log_message_fbs",
        "//aos/starter:starter_fbs",
        "//aos/starter:starter_rpc_fbs",
        "//aos/network:message_bridge_client_fbs",
        "//aos/network:message_bridge_server_fbs",
        "//aos/network:timestamp_fbs",
        "@com_github_foxglove_schemas//:schemas",
    ],
)

aos_config(
    name = "converter_config",
    testonly = True,
    src = "converter_test_config.json",
    flatbuffers = [
        ":vision_fbs",
        "//aos/events:event_loop_fbs",
        "//aos/logging:log_message_fbs",
        "//aos/network:message_bridge_client_fbs",
        "//aos/network:message_bridge_server_fbs",
        "//aos/network:timestamp_fbs",
        "@com_github_foxglove_schemas//:schemas",
    ],
)

cc_test(
    name = "foxglove_image_converter_test",
    srcs = ["foxglove_image_converter_test.cc"],
    data = [
        ":converter_config",
        "@april_tag_test_image",
    ],
    deps = [
        ":foxglove_image_converter_lib",
        "//aos:configuration",
        "//aos/events:simulated_event_loop",
        "//aos/testing:googletest",
        "//aos/testing:path",
        "//aos/testing:tmpdir",
    ],
)

cc_library(
    name = "intrinsics_calibration_lib",
    srcs = [
        "intrinsics_calibration_lib.cc",
    ],
    hdrs = [
        "intrinsics_calibration_lib.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":vision_fbs",
        "//aos/events:event_loop",
        "//frc/control_loops/drivetrain:improved_down_estimator",
        "//frc/vision:charuco_lib",
        "//frc/vision:vision_util_lib",
        "//frc/wpilib:imu_batch_fbs",
        "//frc/wpilib:imu_fbs",
        "//third_party:opencv",
        "@com_google_absl//absl/strings:str_format",
        "@org_tuxfamily_eigen//:eigen",
    ],
)

cc_binary(
    name = "intrinsics_calibration",
    srcs = [
        "intrinsics_calibration.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":intrinsics_calibration_lib",
        ":vision_fbs",
        "//aos:init",
        "//aos/events:shm_event_loop",
        "//frc/control_loops/drivetrain:improved_down_estimator",
        "//frc/vision:charuco_lib",
        "//frc/wpilib:imu_batch_fbs",
        "//frc/wpilib:imu_fbs",
        "//third_party:opencv",
        "@com_google_absl//absl/strings:str_format",
        "@org_tuxfamily_eigen//:eigen",
    ],
)

cc_test(
    name = "intrinsics_calibration_test",
    srcs = [
        "intrinsics_calibration_test.cc",
    ],
    data = [
        "//y2024:aos_config",
        "//y2024/constants/calib_files",
        "@intrinsic_calibration_test_images",
    ],
    shard_count = 4,
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":intrinsics_calibration",
        ":intrinsics_calibration_lib",
        "//aos:init",
        "//aos/events:shm_event_loop",
        "//aos/events:simulated_event_loop",
        "//aos/testing:googletest",
        "//aos/testing:path",
        "//aos/testing:random_seed",
        "//aos/util:math",
        "//third_party:opencv",
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
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_ceres_solver//:ceres",
        "@org_tuxfamily_eigen//:eigen",
    ],
)

cc_test(
    name = "vision_util_lib_test",
    srcs = ["vision_util_lib_test.cc"],
    visibility = ["//visibility:public"],
    deps = [
        "//aos/testing:googletest",
        "//frc/vision:vision_util_lib",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
    ],
)

cc_library(
    name = "target_map_utils",
    srcs = ["target_map_utils.cc"],
    hdrs = ["target_map_utils.h"],
    visibility = ["//visibility:public"],
    deps = [
        "//frc/vision:target_map_fbs",
        "@org_tuxfamily_eigen//:eigen",
    ],
)

cc_library(
    name = "calibrate_multi_cameras_lib",
    srcs = ["calibrate_multi_cameras_lib.cc"],
    hdrs = ["calibrate_multi_cameras_lib.h"],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "//aos:init",
        "//aos/events:simulated_event_loop",
        "//aos/events/logging:log_reader",
        "//aos/util:mcap_logger",
        "//frc/constants:constants_sender_lib",
        "//frc/control_loops:pose",
        "//frc/vision:calibration_fbs",
        "//frc/vision:charuco_lib",
        "//frc/vision:extrinsics_calibration",
        "//frc/vision:target_mapper",
        "//frc/vision:vision_util_lib",
        "//third_party:opencv",
        "@org_tuxfamily_eigen//:eigen",
    ],
)

robot_downloader(
    binaries = [
        ":vision_constants_sender",
        ":turbojpeg_decoder",
        ":apriltag_detector",
        "//aos/events:aos_timing_report_streamer",
        "//aos/util:foxglove_websocket",
        "//y2022/vision:camera_reader",
    ],
    data = [
        ":aos_config",
        ":constants.json",
    ],
    target_compatible_with = ["@platforms//cpu:arm64"],
    target_type = "orin",
)

cc_binary(
    name = "turbojpeg_decoder",
    srcs = ["turbojpeg_decoder.cc"],
    deps = [
        ":vision_fbs",
        "//aos:configuration",
        "//aos:init",
        "//aos/events:shm_event_loop",
        "//third_party:libturbojpeg",
    ],
)

static_flatbuffer(
    name = "camera_constants_fbs",
    srcs = ["camera_constants.fbs"],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":calibration_fbs",
    ],
)

static_flatbuffer(
    name = "camera_constants_list_fbs",
    srcs = ["camera_constants_list.fbs"],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":calibration_fbs",
        ":camera_constants_fbs",
    ],
)

cc_binary(
    name = "apriltag_detector",
    srcs = [
        "apriltag_detector.cc",
    ],
    features = ["cuda"],
    target_compatible_with = ["//tools/platforms/gpu:nvidia"],
    visibility = ["//visibility:public"],
    deps = [
        ":camera_constants_fbs",
        "//aos:configuration",
        "//aos:init",
        "//aos/events:shm_event_loop",
        "//frc/orin:gpu_apriltag_lib",
        "//third_party:cudart",
        "//third_party/apriltag",
        "@com_github_nvidia_cccl//:cccl",
        "@com_google_absl//absl/flags:flag",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
    ],
)

cc_binary(
    name = "vision_constants_sender",
    srcs = [
        "vision_constants_sender.cc",
    ],
    deps = [
        ":camera_constants_fbs",
        ":camera_constants_list_fbs",
        "//aos:init",
        "//aos/events:shm_event_loop",
        "//frc/constants:constants_sender_lib",
    ],
)

jinja2_template(
    name = "constants.json",
    src = "constants.jinja2.json",
    includes = glob([
        "constants/*.json",
    ]),
)
