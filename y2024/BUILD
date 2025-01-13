load("//aos:config.bzl", "aos_config")

aos_config(
    name = "aos_config",
    src = "y2024.json",
    flatbuffers = [
        "//aos/network:message_bridge_client_fbs",
        "//aos/network:message_bridge_server_fbs",
        "//aos/network:timestamp_fbs",
        "//frc971/input:robot_state_fbs",
        "//frc971/vision:vision_fbs",
        "//frc971/vision:target_map_fbs",
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
    name = "config_imu",
    src = "y2024_imu.json",
    flatbuffers = [
        "//aos/network:message_bridge_client_fbs",
        "//aos/network:message_bridge_server_fbs",
        "//frc971/imu_fdcan:dual_imu_fbs",
        "//frc971/imu_fdcan:can_translator_status_fbs",
        "//frc971/imu_fdcan:dual_imu_blender_status_fbs",
        "//frc971/control_loops/drivetrain/localization:localizer_output_fbs",
        "//frc971/can_logger:can_logging_fbs",
        "//frc971/orin:hardware_stats_fbs",
        "//aos/network:timestamp_fbs",
        "//aos/util:filesystem_fbs",
        "//aos/network:remote_message_fbs",
        "//frc971/vision:calibration_fbs",
        "//frc971/vision:target_map_fbs",
        "//frc971/vision:vision_fbs",
        "@com_github_foxglove_schemas//:schemas",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "//aos/events:aos_config",
        "//frc971/control_loops/drivetrain:aos_config",
    ],
)

aos_config(
    name = "config_roborio",
    src = "y2024_roborio.json",
    flatbuffers = [
        "//frc971:can_configuration_fbs",
        "//aos/network:remote_message_fbs",
        "//aos/network:message_bridge_client_fbs",
        "//aos/network:message_bridge_server_fbs",
        "//frc971/wpilib:pdp_values_fbs",
        # y2019 stuff shouldn't be here (e.g. target selector)
        "//aos/network:timestamp_fbs",
        "//frc971/control_loops/drivetrain:drivetrain_can_position_fbs",
        "//frc971/control_loops/drivetrain:rio_localizer_inputs_fbs",
        "//frc971/control_loops/drivetrain/localization:localizer_output_fbs",
        "//frc971/can_logger:can_logging_fbs",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "//aos/events:aos_config",
        "//frc971/autonomous:aos_config",
        "//frc971/control_loops/drivetrain:aos_config",
        "//frc971/input:aos_config",
    ],
)

aos_config(
    name = "config_orin1",
    src = "y2024_orin1.json",
    flatbuffers = [
        "//aos/network:message_bridge_client_fbs",
        "//aos/network:message_bridge_server_fbs",
        "//aos/network:timestamp_fbs",
        "//aos/network:remote_message_fbs",
        "//frc971/control_loops/drivetrain/localization:localizer_output_fbs",
        "//frc971/orin:hardware_stats_fbs",
        "//frc971/vision:calibration_fbs",
        "//frc971/vision:target_map_fbs",
        "//frc971/vision:vision_fbs",
        "//aos/util:filesystem_fbs",
        "@com_github_foxglove_schemas//:schemas",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "//aos/events:aos_config",
        "//frc971/control_loops/drivetrain:aos_config",
        "//frc971/input:aos_config",
    ],
)
