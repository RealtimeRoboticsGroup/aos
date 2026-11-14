load("@aos//aos/flatbuffers:ts.bzl", "flatbuffer_ts_library")
load("@rules_cc//cc:cc_binary.bzl", "cc_binary")
load("//aos:config.bzl", "aos_config")
load("//aos/flatbuffers:build_defs.bzl", "DEFAULT_FLATC_ARGS")
load("//aos/flatbuffers:generate.bzl", "static_flatbuffer")
load("//frc:downloader.bzl", "robot_downloader")
load("//frc/downloader:downloader.bzl", "aos_downloader_dir")
load("//tools/build_rules:template.bzl", "jinja2_template")
load(":validator.bzl", "camera_constants_json")

static_flatbuffer(
    name = "vision_fbs",
    srcs = ["vision.fbs"],
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

static_flatbuffer(
    name = "target_map_fbs",
    srcs = ["target_map.fbs"],
    visibility = ["//visibility:public"],
)

aos_config(
    name = "aos_config",
    src = "single_node_camera_config.json",
    flatbuffers = [
        ":coral_detection_fbs",
        ":game_piece_locations_fbs",
        ":vision_fbs",
        ":target_map_fbs",
        ":camera_settings_fbs",
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
        ":chassis_speeds_fbs",
        ":pose2d_fbs",
        "//frc/vision/field_map:visualization_fbs",
        "//frc/vision/field_map:status_fbs",
        "//frc/vision/field_map:udp_status_fbs",
        "//frc/control_loops/drivetrain/localization:localizer_output_fbs",
        "//frc/control_loops/drivetrain:localizer_fbs",
        "//frc/input:joystick_state_fbs",
        "//frc/input:robot_state_fbs",
        "@com_github_foxglove_schemas//:schemas",
    ],
    visibility = ["//visibility:public"],
)

aos_downloader_dir(
    name = "base_intrinsics_files",
    srcs = glob(["constants/calibration*.json"]),
    dir = "base_intrinsics",
    visibility = ["//visibility:public"],
)

robot_downloader(
    binaries = [
        "//frc/vision/detection:yolo",
        ":image_dump_cli",
        "//frc/vision/detection:apriltag_detector",
        ":field_side_exposure_adjuster",
        ":game_piece_mapper",
        "//frc/vision/camera:image_logger",
        "//frc/vision/calibration:intrinsics_calibration",
        "//frc/vision/camera:turbojpeg_decoder",
        "//frc/vision/networking:network_tables_publisher",
        "//frc/vision/networking:network_tables_client",
        "//frc/vision/networking:vision_constants_sender",
        "//aos/events:aos_timing_report_streamer",
        "//aos/network:web_proxy_main",
        "//aos/starter:irq_affinity",
        "//aos/events/logging:log_cat",
        "//aos/util:filesystem_monitor",
        "//aos/util:foxglove_websocket",
        "//frc/orin:hardware_monitor",
        "//frc/orin:localizer_logger",
        "//frc/vision/field_map:field_map_constants_sender",
        "//frc/vision/field_map:localizer_main",
        "//frc/vision/networking:network_tables_swerve_client",
        "//frc/vision/camera:camera_reader",
    ],
    data = [
        ":aos_config",
        ":constants.json",
        "//frc/orin:orin_irq_config.json",
        "@frc2026_field_map_welded//file:frc2026.fmap",
    ],
    # Deploy the raw intrinsics files so that it is easy to run intrinsics
    # without needing to manually scp around intrinsics JSON's.
    dirs = [
        ":base_intrinsics_files",
        "//frc/vision/field_map/www:www_files",
    ],
    target_compatible_with = [
        "@platforms//cpu:arm64",
        "@platforms//os:linux",
    ],
    target_type = "orin",
)

static_flatbuffer(
    name = "turbojpeg_decoder_status_fbs",
    srcs = ["turbojpeg_decoder_status.fbs"],
    visibility = ["//visibility:public"],
)

static_flatbuffer(
    name = "camera_settings_fbs",
    srcs = ["camera_settings.fbs"],
    visibility = ["//visibility:public"],
    deps = [
        ":calibration_fbs",
    ],
)

static_flatbuffer(
    name = "camera_constants_fbs",
    srcs = ["camera_constants.fbs"],
    visibility = ["//visibility:public"],
    deps = [
        ":calibration_fbs",
        ":camera_settings_fbs",
    ],
)

static_flatbuffer(
    name = "camera_constants_list_fbs",
    srcs = ["camera_constants_list.fbs"],
    visibility = ["//visibility:public"],
    deps = [
        ":calibration_fbs",
        ":camera_constants_fbs",
    ],
)

jinja2_template(
    name = "constants_unvalidated.json",
    src = "templates/constants.jinja2.json",
    includes = glob([
        "constants/*.json",
    ]),
)

camera_constants_json(
    name = "camera_constants_json",
    src = ":constants_unvalidated.json",
    out = "constants.json",
)

static_flatbuffer(
    name = "field_map_fbs",
    srcs = ["field_map.fbs"],
    # field_map.fbs mirrors the Limelight .fmap JSON format, so its pngBase64
    # field can't be renamed to snake_case to quiet flatc's warning.
    flatc_args = DEFAULT_FLATC_ARGS + ["--no-warnings"],
    visibility = ["//visibility:public"],
)

cc_binary(
    name = "field_side_exposure_adjuster",
    srcs = ["field_side_exposure_adjuster.cc"],
    visibility = ["//visibility:public"],
    deps = [
        ":camera_settings_fbs",
        "//aos:configuration",
        "//aos:init",
        "//aos/events:shm_event_loop",
        "//frc/input:joystick_state_fbs",
    ],
)

static_flatbuffer(
    name = "coral_detection_fbs",
    srcs = ["coral_detection.fbs"],
    visibility = ["//visibility:public"],
)

flatbuffer_ts_library(
    name = "game_piece_locations_ts_fbs",
    srcs = ["game_piece_locations.fbs"],
    target_compatible_with = ["@platforms//os:linux"],
    ts_files = [
        "frc/vision.ts",
        "frc/vision/game-piece-locations.ts",
        "frc/vision/game-piece-location.ts",
    ],
    visibility = ["//visibility:public"],
)

static_flatbuffer(
    name = "game_piece_locations_fbs",
    srcs = ["game_piece_locations.fbs"],
    visibility = ["//visibility:public"],
)

cc_binary(
    name = "game_piece_mapper",
    srcs = [
        "game_piece_mapper.cc",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":camera_constants_fbs",
        ":coral_detection_fbs",
        ":game_piece_locations_fbs",
        "//aos:configuration",
        "//aos:init",
        "//aos/events:shm_event_loop",
        "//frc/constants:constants_sender_lib",
        "//frc/control_loops/drivetrain/localization:localizer_output_fbs",
        "@com_google_absl//absl/flags:flag",
        "@com_google_absl//absl/log:absl_check",
        "@com_google_absl//absl/log:absl_log",
        "@eigen",
    ],
)

cc_binary(
    name = "image_dump",
    srcs = [
        "image_dump.cc",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":vision_fbs",
        "//aos:init",
        "//aos:sha256",
        "//aos/events:simulated_event_loop",
        "//aos/events/logging:log_reader",
        "//aos/util:file",
        "@com_google_absl//absl/flags:flag",
        "@com_google_absl//absl/log:absl_check",
        "@com_google_absl//absl/log:absl_log",
    ],
)

cc_binary(
    name = "image_dump_cli",
    srcs = [
        "image_dump_cli.cc",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":vision_fbs",
        "//aos:init",
        "//aos:sha256",
        "//aos/events:shm_event_loop",
        "//aos/util:file",
        "@com_google_absl//absl/flags:flag",
        "@com_google_absl//absl/log:absl_check",
        "@com_google_absl//absl/log:absl_log",
    ],
)

static_flatbuffer(
    name = "chassis_speeds_fbs",
    srcs = [
        "chassis_speeds.fbs",
    ],
    visibility = ["//visibility:public"],
)

static_flatbuffer(
    name = "pose2d_fbs",
    srcs = [
        "pose2d.fbs",
    ],
    visibility = ["//visibility:public"],
)
