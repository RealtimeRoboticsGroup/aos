load("//aos:config.bzl", "aos_config")
load("//aos/util:config_validator_macro.bzl", "config_validator_test")
load("//frc:downloader.bzl", "robot_downloader")

config_validator_test(
    name = "config_validator_test",
    config = "//y2024_swerve:aos_config",
)

robot_downloader(
    binaries = [
        "//aos/network:web_proxy_main",
        "//aos/events/logging:log_cat",
        "//y2024_swerve/constants:constants_sender",
        "//y2024_swerve/control_loops/superstructure",
        "//aos/events:aos_timing_report_streamer",
        ":joystick_reader",
        "//aos/events/logging:logger_main",
        "//aos/starter:irq_affinity",
        ":wpilib_interface",
        ":swerve_publisher",
        "//y2024_swerve/control_loops:swerve_control_loops",
        "//frc/can_logger",
        "//aos/network:message_bridge_client",
        "//aos/network:message_bridge_server",
    ],
    data = [
        ":aos_config",
        ":swerve_publisher_goal_json",
        "//y2024_swerve/constants:constants.json",
        "@ctre_phoenix6_api_cpp_athena//:shared_libraries",
        "@ctre_phoenix6_tools_athena//:shared_libraries",
        "@ctre_phoenix_api_cpp_athena//:shared_libraries",
        "@ctre_phoenix_cci_athena//:shared_libraries",
    ],
    dirs = [
    ],
    target_compatible_with = ["@platforms//os:linux"],
)

robot_downloader(
    name = "orin_download",
    binaries = [
        "//frc/wpilib:joystick_republish",
        "//aos/events:aos_timing_report_streamer",
        "//aos/events/logging:log_cat",
        "//aos:aos_jitter",
        "//aos/network:web_proxy_main",
        "//aos/starter:irq_affinity",
        "//aos/util:foxglove_websocket",
        "//frc/image_streamer:image_streamer",
        "//frc/orin:hardware_monitor",
        "//frc/orin:argus_monitor",
        "//frc/vision:intrinsics_calibration",
        "//aos/util:filesystem_monitor",
        "//y2024_swerve/constants:constants_sender",
        "//frc/vision:foxglove_image_converter",
        "//aos/events/logging:logger_main",
        "//frc/imu_fdcan:can_translator",
        "//frc/imu_fdcan:dual_imu_blender",
        "//frc/orin:localizer_logger",
        "//frc/can_logger",
        "//aos/network:message_bridge_client",
        "//aos/network:message_bridge_server",
        "//frc/orin:argus_camera",
        "//y2024_swerve/vision:apriltag_detector",
        "//frc/vision:image_logger",
    ],
    data = [
        ":aos_config",
        "//frc/orin:orin_irq_config.json",
        "//y2024_swerve/constants:constants.json",
    ],
    dirs = [
    ],
    target_compatible_with = ["//tools/platforms/hardware:raspberry_pi"],
    target_type = "pi",
)

filegroup(
    name = "swerve_publisher_goal_json",
    srcs = [
        "swerve_drivetrain_goal.json",
    ],
    visibility = ["//y2024_swerve:__subpackages__"],
)

cc_library(
    name = "constants",
    srcs = ["constants.cc"],
    hdrs = [
        "constants.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//aos/mutex",
        "//aos/network:team_number",
        "//frc:constants",
        "//y2024_swerve/constants:constants_fbs",
        "@com_google_absl//absl/base",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
    ],
)

cc_binary(
    name = "swerve_publisher",
    srcs = ["swerve_publisher_main.cc"],
    deps = [
        ":swerve_publisher_lib",
        "//aos/events:shm_event_loop",
        "@com_google_absl//absl/flags:flag",
    ],
)

cc_library(
    name = "swerve_publisher_lib",
    srcs = ["swerve_publisher_lib.cc"],
    hdrs = ["swerve_publisher_lib.h"],
    deps = [
        "//aos:init",
        "//aos/events:event_loop",
        "//frc/control_loops/swerve:swerve_drivetrain_goal_fbs",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
    ],
)

cc_test(
    name = "swerve_publisher_lib_test",
    srcs = [
        "swerve_publisher_lib_test.cc",
    ],
    data = [
        ":aos_config",
        ":swerve_publisher_goal_json",
    ],
    deps = [
        ":swerve_publisher_lib",
        "//aos/events:simulated_event_loop",
        "//aos/testing:googletest",
    ],
)

cc_binary(
    name = "wpilib_interface",
    srcs = ["wpilib_interface.cc"],
    target_compatible_with = ["//tools/platforms/hardware:roborio"],
    deps = [
        ":constants",
        "//aos:init",
        "//aos/events:shm_event_loop",
        "//frc/constants:constants_sender_lib",
        "//frc/control_loops:control_loops_fbs",
        "//frc/control_loops/swerve:swerve_drivetrain_can_position_fbs",
        "//frc/control_loops/swerve:swerve_drivetrain_position_fbs",
        "//frc/queues:gyro_fbs",
        "//frc/wpilib:can_sensor_reader",
        "//frc/wpilib:joystick_sender",
        "//frc/wpilib:pdp_fetcher",
        "//frc/wpilib:sensor_reader",
        "//frc/wpilib:talonfx",
        "//frc/wpilib:wpilib_robot_base",
        "//frc/wpilib/swerve:swerve_drivetrain_writer",
        "//y2024_swerve/constants:constants_fbs",
    ],
)

aos_config(
    name = "aos_config",
    src = "y2024_swerve.json",
    flatbuffers = [
        "//aos/network:message_bridge_client_fbs",
        "//aos/network:message_bridge_server_fbs",
        "//aos/network:timestamp_fbs",
        "//frc/input:robot_state_fbs",
        "//frc/vision:vision_fbs",
        "//frc/vision:target_map_fbs",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":config_imu",
        ":config_orin1",
        ":config_roborio",
    ],
)

aos_config(
    name = "config_roborio",
    src = "y2024_swerve_roborio.json",
    flatbuffers = [
        "//frc:can_configuration_fbs",
        "//frc/queues:gyro_fbs",
        "//aos/network:remote_message_fbs",
        "//aos/network:message_bridge_client_fbs",
        "//aos/network:message_bridge_server_fbs",
        "//aos/network:timestamp_fbs",
        "//frc/control_loops/swerve:swerve_drivetrain_goal_fbs",
        "//frc/control_loops/swerve:swerve_drivetrain_status_fbs",
        "//frc/control_loops/swerve:swerve_drivetrain_output_fbs",
        "//frc/control_loops/swerve:swerve_drivetrain_position_fbs",
        "//frc/control_loops/swerve:swerve_drivetrain_joystick_goal_fbs",
        "//y2024_swerve/constants:constants_fbs",
        "//frc/control_loops/swerve:swerve_drivetrain_can_position_fbs",
        "//frc/control_loops/drivetrain:drivetrain_can_position_fbs",
        "//frc/can_logger:can_logging_fbs",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "//aos/events:aos_config",
        "//frc/autonomous:aos_config",
        "//frc/control_loops/drivetrain:aos_config",
        "//frc/input:aos_config",
        "//frc/wpilib:aos_config",
    ],
)

aos_config(
    name = "config_imu",
    src = "y2024_swerve_imu.json",
    flatbuffers = [
        "//aos/network:message_bridge_client_fbs",
        "//y2024_swerve/constants:constants_fbs",
        "//aos/network:message_bridge_server_fbs",
        "//frc/imu_fdcan:dual_imu_fbs",
        "//frc/imu_fdcan:can_translator_status_fbs",
        "//frc/imu_fdcan:dual_imu_blender_status_fbs",
        "//frc/can_logger:can_logging_fbs",
        "//frc/orin:hardware_stats_fbs",
        "//aos/network:timestamp_fbs",
        "//aos/util:filesystem_fbs",
        "//aos/network:remote_message_fbs",
        "//frc/vision:calibration_fbs",
        "//frc/vision:target_map_fbs",
        "//frc/vision:vision_fbs",
        "@com_github_foxglove_schemas//:schemas",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "//aos/events:aos_config",
        "//frc/control_loops/drivetrain:aos_config",
    ],
)

aos_config(
    name = "config_orin1",
    src = "y2024_swerve_orin1.json",
    flatbuffers = [
        "//aos/network:message_bridge_client_fbs",
        "//aos/network:message_bridge_server_fbs",
        "//aos/network:timestamp_fbs",
        "//aos/network:remote_message_fbs",
        "//y2024_swerve/constants:constants_fbs",
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

cc_binary(
    name = "joystick_reader",
    srcs = [
        ":joystick_reader.cc",
    ],
    deps = [
        "//aos:init",
        "//aos/actions:action_lib",
        "//aos/events:shm_event_loop",
        "//aos/logging",
        "//frc/input:drivetrain_input",
        "//frc/input:joystick_input",
        "//frc/input:redundant_joystick_data",
        "//frc/input:swerve_joystick_input",
    ],
)
