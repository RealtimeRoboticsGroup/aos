load("@com_github_google_flatbuffers//:typescript.bzl", "flatbuffer_ts_library")
load("//aos/flatbuffers:generate.bzl", "static_flatbuffer")
load("//tools/build_rules:js.bzl", "ts_project")

ts_project(
    name = "localizer_plotter",
    srcs = ["localizer_plotter.ts"],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "//aos/network/www:aos_plotter",
        "//aos/network/www:colors",
        "//aos/network/www:proxy",
        "//frc/wpilib:imu_plot_utils",
    ],
)

static_flatbuffer(
    name = "udp_status_fbs",
    srcs = [
        "udp_status.fbs",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
)

static_flatbuffer(
    name = "status_fbs",
    srcs = [
        "status.fbs",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "//frc/math:matrix_fbs",
    ],
)

flatbuffer_ts_library(
    name = "status_ts_fbs",
    srcs = ["status.fbs"],
    visibility = ["//visibility:public"],
    deps = [
        "//frc/control_loops/drivetrain:drivetrain_status_ts_fbs",
        "//frc/imu_reader:imu_failures_ts_fbs",
        "//frc/math:matrix_ts_fbs",
    ],
)

static_flatbuffer(
    name = "visualization_fbs",
    srcs = [
        "visualization.fbs",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":status_fbs",
    ],
)

flatbuffer_ts_library(
    name = "visualization_ts_fbs",
    srcs = ["visualization.fbs"],
    visibility = ["//visibility:public"],
    deps = [
        ":status_ts_fbs",
    ],
)

cc_library(
    name = "localizer",
    srcs = ["localizer.cc"],
    hdrs = [
        "hybrid_ekf.h",
        "localizer.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":chassis_speeds_fbs",
        ":pose2d_fbs",
        ":status_fbs",
        ":visualization_fbs",
        "//aos/containers:sized_array",
        "//aos/events:event_loop",
        "//aos/network:message_bridge_client_fbs",
        "//frc/constants:constants_sender_lib",
        "//frc/control_loops:pose",
        "//frc/control_loops/drivetrain:hybrid_ekf",
        "//frc/control_loops/drivetrain:localizer_fbs",
        "//frc/control_loops/drivetrain/localization:localizer_output_fbs",
        "//frc/control_loops/drivetrain/localization:utils",
        "//frc/math:flatbuffers_matrix",
        "//frc/vision:camera_constants_fbs",
        "//frc/vision:target_map_fbs",
        "//frc/vision:target_map_utils",
    ],
)

cc_test(
    name = "localizer_test",
    srcs = ["localizer_test.cc"],
    data = ["//frc/vision:aos_config"],
    deps = [
        ":localizer",
        ":status_fbs",
        "//aos/events:simulated_event_loop",
        "//aos/events/logging:log_writer",
        "//aos/testing:googletest",
        "//frc/control_loops/drivetrain:drivetrain_test_lib",
        "//frc/control_loops/drivetrain:localizer_fbs",
        "//frc/control_loops/drivetrain/localization:utils",
    ],
)

cc_binary(
    name = "localizer_main",
    srcs = ["localizer_main.cc"],
    visibility = ["//visibility:public"],
    deps = [
        ":localizer",
        "//aos:init",
        "//aos/events:shm_event_loop",
        "//frc/constants:constants_sender_lib",
    ],
)

ts_project(
    name = "corrections_plotter",
    srcs = ["corrections_plotter.ts"],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":visualization_ts_fbs",
        "//aos/network/www:aos_plotter",
        "//aos/network/www:colors",
        "//aos/network/www:proxy",
    ],
)

cc_binary(
    name = "localizer_replay",
    srcs = ["localizer_replay.cc"],
    data = [
        "//frc/vision:aos_config",
        "//frc/vision:constants.json",
        "//frc/vision/swerve_localizer/www:www_directory",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":localizer",
        ":simulated_constants_sender_lib",
        "//aos:configuration",
        "//aos:init",
        "//aos:json_to_flatbuffer",
        "//aos/events:simulated_event_loop",
        "//aos/events/logging:log_reader",
        "//aos/events/logging:log_writer",
        "//aos/network:web_proxy",
        "//aos/util:simulation_logger",
        "//frc/constants:constants_sender_lib",
        "//frc/imu_fdcan:dual_imu_blender_lib",
    ],
)

cc_library(
    name = "simulated_constants_sender_lib",
    hdrs = ["simulated_constants_sender_lib.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":field_map_constants_sender_lib",
        "//aos/events:simulated_event_loop",
        "//frc/vision:camera_constants_fbs",
        "//frc/vision:camera_constants_list_fbs",
        "//frc/vision:field_map_fbs",
    ],
)

cc_library(
    name = "field_map_constants_sender_lib",
    srcs = ["field_map_constants_sender_lib.cc"],
    hdrs = ["field_map_constants_sender_lib.h"],
    data = [
        "@frc2025_field_map_welded//file:frc2025r2.fmap",
    ],
    deps = [
        "//aos:json_to_flatbuffer",
        "//aos/events:event_loop",
        "//frc/vision:field_map_fbs",
        "//frc/vision:target_map_fbs",
        "@org_tuxfamily_eigen//:eigen",
    ],
)

cc_binary(
    name = "field_map_constants_sender",
    srcs = [
        "field_map_constants_sender.cc",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":field_map_constants_sender_lib",
        "//aos:configuration",
        "//aos:init",
        "//aos:json_to_flatbuffer",
        "//aos/events:shm_event_loop",
        "//frc/vision:field_map_fbs",
    ],
)

static_flatbuffer(
    name = "chassis_speeds_fbs",
    srcs = [
        "chassis_speeds.fbs",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
)

static_flatbuffer(
    name = "pose2d_fbs",
    srcs = [
        "pose2d.fbs",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
)

cc_binary(
    name = "network_tables_swerve_client",
    srcs = [
        "network_tables_swerve_client.cc",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":chassis_speeds_fbs",
        ":pose2d_fbs",
        ":udp_status_fbs",
        "//aos:configuration",
        "//aos:init",
        "//aos:json_to_flatbuffer",
        "//aos/events:shm_event_loop",
        "//aos/vision/events:udp",
        "//frc/constants:constants_sender_lib",
        "//frc/control_loops/drivetrain/localization:localizer_output_fbs",
        "//frc/input:joystick_state_fbs",
        "//frc/input:robot_state_fbs",
        "//frc/vision:field_map_fbs",
        "//frc/vision:game_piece_locations_fbs",
        "//frc/vision:target_map_fbs",
        "@com_github_wpilibsuite_allwpilib//ntcore:ntcore.static",
        "@com_github_wpilibsuite_allwpilib//wpimath:wpimath.static",
        "@com_google_absl//absl/log:die_if_null",
        "@org_tuxfamily_eigen//:eigen",
    ],
)
