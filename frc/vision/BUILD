load("@com_github_google_flatbuffers//:typescript.bzl", "flatbuffer_ts_library")
load("//aos:config.bzl", "aos_config")
load("//aos/flatbuffers:generate.bzl", "static_flatbuffer")
load("//frc:downloader.bzl", "robot_downloader")
load("//frc/downloader:downloader.bzl", "aos_downloader_dir")
load("//tools/build_rules:template.bzl", "jinja2_template")
load(":validator.bzl", "camera_constants_json")

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
    ts_files = [
        "frc/vision.ts",
        "frc/vision/image-format.ts",
        "frc/vision/camera-image.ts",
    ],
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
    ts_files = [
        "frc/vision/calibration.ts",
        "frc/vision/calibration/transformation-matrix.ts",
        "frc/vision/calibration/camera-calibration.ts",
        "frc/vision/calibration/calibration-data.ts",
    ],
    visibility = ["//visibility:public"],
)

flatbuffer_ts_library(
    name = "target_map_ts_fbs",
    srcs = ["target_map.fbs"],
    target_compatible_with = ["@platforms//os:linux"],
    ts_files = [
        "frc/vision.ts",
        "frc/vision/position.ts",
        "frc/vision/quaternion.ts",
        "frc/vision/target-pose-fbs.ts",
        "frc/vision/target-map.ts",
    ],
    visibility = ["//visibility:public"],
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
        ":camera_constants_fbs",
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
    name = "cuda_camera_image_callback",
    srcs = ["cuda_camera_image_callback.cc"],
    hdrs = ["cuda_camera_image_callback.h"],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":charuco_lib",
        "//aos/events:shm_event_loop",
        "//frc/orin:cuda",
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
        ":turbojpeg_decoder_status_fbs",
        "//aos/events:event_loop_fbs",
        "//aos/logging:log_message_fbs",
        "//aos/starter:starter_fbs",
        "//aos/util:filesystem_fbs",
        "//aos/starter:starter_rpc_fbs",
        "//aos/network:message_bridge_client_fbs",
        "//aos/network:message_bridge_server_fbs",
        "//aos/network:timestamp_fbs",
        "//frc/orin:hardware_stats_fbs",
        "//frc/vision/swerve_localizer:chassis_speeds_fbs",
        "//frc/vision/swerve_localizer:pose2d_fbs",
        "//frc/input:joystick_state_fbs",
        "//frc/input:robot_state_fbs",
        "@com_github_foxglove_schemas//:schemas",
    ],
    visibility = ["//frc/vision:__subpackages__"],
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
        "@com_google_absl//absl/log:die_if_null",
        "@com_google_absl//absl/strings:str_format",
        "@org_tuxfamily_eigen//:eigen",
    ],
)

cc_binary(
    name = "intrinsics_calibration",
    srcs = [
        "intrinsics_calibration.cc",
    ],
    data = [":aos_config"],
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

aos_config(
    name = "intrinsics_test_config",
    src = "intrinsics_test_config_source.json",
    flatbuffers = [
        "//aos/network:message_bridge_client_fbs",
        "//aos/network:message_bridge_server_fbs",
        "//aos/network:timestamp_fbs",
        "//aos/network:remote_message_fbs",
        "//frc/control_loops/drivetrain/localization:localizer_output_fbs",
        "//frc/orin:hardware_stats_fbs",
        "//frc/vision:calibration_fbs",
        "//frc/vision:target_map_fbs",
        "//frc/vision:vision_fbs",
        "//aos/util:filesystem_fbs",
        "@com_github_foxglove_schemas//:schemas",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "//aos/events:aos_config",
        "//frc/control_loops/drivetrain:aos_config",
        "//frc/input:aos_config",
    ],
)

cc_test(
    name = "intrinsics_calibration_test",
    srcs = [
        "intrinsics_calibration_test.cc",
    ],
    data = [
        ":intrinsics_test_config",
        "//frc/vision/test_calib_files",
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

aos_downloader_dir(
    name = "base_intrinsics_files",
    srcs = glob(["constants/calibration*.json"]),
    dir = "base_intrinsics",
)

robot_downloader(
    binaries = [
        ":apriltag_detector",
        ":image_logger",
        ":intrinsics_calibration",
        ":turbojpeg_decoder",
        ":network_tables_publisher",
        ":vision_constants_sender",
        "//aos/events:aos_timing_report_streamer",
        "//aos/starter:irq_affinity",
        "//aos/util:filesystem_monitor",
        "//aos/util:foxglove_websocket",
        "//frc/orin:hardware_monitor",
        "//frc/orin:localizer_logger",
        "//frc/vision/swerve_localizer:network_tables_swerve_client",
        "//frc/vision:camera_reader",
    ],
    data = [
        ":aos_config",
        ":constants.json",
        "//frc/orin:orin_irq_config.json",
        "@frc2025_field_map_welded//file:frc2025r2.fmap",
    ],
    # Deploy the raw intrinsics files so that it is easy to run intrinsics
    # without needing to manually scp around intrinsics JSON's.
    dirs = [":base_intrinsics_files"],
    target_compatible_with = ["@platforms//cpu:arm64"],
    target_type = "orin",
)

static_flatbuffer(
    name = "turbojpeg_decoder_status_fbs",
    srcs = ["turbojpeg_decoder_status.fbs"],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
)

cc_binary(
    name = "turbojpeg_decoder",
    srcs = ["turbojpeg_decoder.cc"],
    deps = [
        ":turbojpeg_decoder_status_fbs",
        ":vision_fbs",
        "//aos:configuration",
        "//aos:init",
        "//aos/containers:inlined_vector",
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
    name = "constants_unvalidated.json",
    src = "constants.jinja2.json",
    includes = glob([
        "constants/*.json",
    ]),
)

cc_binary(
    name = "camera_reader",
    srcs = [
        "camera_reader_main.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "//aos:init",
        "//aos/events:shm_event_loop",
        "//frc/constants:constants_sender_lib",
        "//frc/vision:v4l2_reader",
    ],
)

cc_binary(
    name = "camera_constants_formatter",
    srcs = ["camera_constants_formatter.cc"],
    visibility = ["//visibility:public"],
    deps = [
        ":camera_constants_list_fbs",
        "//aos:init",
        "//aos:json_to_flatbuffer",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
    ],
)

camera_constants_json(
    name = "camera_constants_json",
    src = ":constants_unvalidated.json",
    out = "constants.json",
)

static_flatbuffer(
    name = "field_map_fbs",
    srcs = ["field_map.fbs"],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
)

cc_binary(
    name = "network_tables_publisher",
    srcs = ["network_tables_publisher.cc"],
    deps = [
        ":camera_constants_fbs",
        ":field_map_fbs",
        ":target_map_fbs",
        "//aos:configuration",
        "//aos:init",
        "//aos/events:shm_event_loop",
        "//frc/constants:constants_sender_lib",
        "@com_github_wpilibsuite_allwpilib//ntcore:ntcore.static",
        "@com_github_wpilibsuite_allwpilib//wpimath:wpimath.static",
        "@com_google_absl//absl/flags:flag",
        "@org_tuxfamily_eigen//:eigen",
    ],
)

cc_binary(
    name = "robot_transformation_calculator",
    srcs = [
        "robot_transformation_calculator.cc",
    ],
    deps = [
        "//aos:init",
        "@com_google_absl//absl/log",
        "@org_tuxfamily_eigen//:eigen",
    ],
)

cc_binary(
    name = "network_tables_server",
    srcs = ["network_tables_server.cc"],
    deps = [
        "//aos:init",
        "@com_github_wpilibsuite_allwpilib//ntcore:ntcore.static",
        "@com_google_absl//absl/flags:flag",
    ],
)

cc_binary(
    name = "network_tables_client",
    srcs = ["network_tables_client.cc"],
    deps = [
        "//aos:init",
        "@com_github_wpilibsuite_allwpilib//ntcore:ntcore.static",
        "@com_github_wpilibsuite_allwpilib//wpimath:wpimath.static",
        "@com_google_absl//absl/flags:flag",
        "@com_google_absl//absl/log:die_if_null",
    ],
)

cc_binary(
    name = "calibrate_multi_cameras",
    srcs = [
        "calibrate_multi_cameras.cc",
        "vision_util.cc",
        "vision_util.h",
    ],
    data = [
        ":aos_config",
        ":constants.json",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "//aos:init",
        "//aos/events:simulated_event_loop",
        "//aos/events/logging:log_reader",
        "//aos/util:mcap_logger",
        "//frc/constants:constants_sender_lib",
        "//frc/control_loops:pose",
        "//frc/vision:calibrate_multi_cameras_lib",
        "//frc/vision:calibration_fbs",
        "//frc/vision:charuco_lib",
        "//frc/vision:extrinsics_calibration",
        "//frc/vision:target_mapper",
        "//frc/vision:vision_util_lib",
        "//frc/vision/swerve_localizer:simulated_constants_sender_lib",
        "//third_party:opencv",
        "@org_tuxfamily_eigen//:eigen",
    ],
)

cc_binary(
    name = "yolo",
    srcs = [
        "yolo.cc",
    ],
    features = ["cuda"],
    visibility = ["//visibility:public"],
    deps = [
        "//aos:init",
        "//aos/util:file",
        "//frc/orin:cuda",
        "//third_party:cudart",
        "//third_party:tensorrt",
        "@com_google_absl//absl/flags:flag",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
    ],
)
