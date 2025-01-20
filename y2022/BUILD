load("//aos:config.bzl", "aos_config")

aos_config(
    name = "aos_config",
    src = "y2022.json",
    flatbuffers = [
        "//aos/network:message_bridge_client_fbs",
        "//aos/network:message_bridge_server_fbs",
        "//aos/network:timestamp_fbs",
        "//frc/input:robot_state_fbs",
        "//frc/vision:vision_fbs",
        "//frc/vision:calibration_fbs",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":config_imu",
        ":config_roborio",
    ],
)

aos_config(
    name = "config_imu",
    src = "y2022_imu.json",
    flatbuffers = [
        "//aos/network:message_bridge_client_fbs",
        "//aos/network:message_bridge_server_fbs",
        "//aos/network:timestamp_fbs",
        "//aos/network:remote_message_fbs",
        "//frc/control_loops/drivetrain/localization:localizer_output_fbs",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "//aos/events:aos_config",
        "//frc/control_loops/drivetrain:aos_config",
    ],
)

aos_config(
    name = "config_roborio",
    src = "y2022_roborio.json",
    flatbuffers = [
        "//aos/network:remote_message_fbs",
        "//aos/network:message_bridge_client_fbs",
        "//aos/network:message_bridge_server_fbs",
        "//aos/network:timestamp_fbs",
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

py_library(
    name = "python_init",
    srcs = ["__init__.py"],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
)
