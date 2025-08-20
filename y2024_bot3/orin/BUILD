cc_binary(
    name = "ctre_interface",
    srcs = ["ctre_interface.cc"],
    target_compatible_with = ["@platforms//cpu:arm64"],
    visibility = ["//visibility:public"],
    deps = [
        "//aos:init",
        "//frc/control_loops/swerve:swerve_drivetrain_can_position_fbs",
        "//frc/control_loops/swerve:swerve_drivetrain_output_fbs",
        "//frc/wpilib:can_sensor_reader",
        "//frc/wpilib:loop_output_handler",
        "//frc:can_configuration_fbs",
        "//frc/wpilib:talonfx",
        "//y2024_bot3:constants",
        "//frc/constants:constants_sender_lib",
        "//y2024_bot3/constants:constants_fbs",
    ],
)

cc_binary(
    name = "can_logger",
    srcs = [
        "can_logger_main.cc",
    ],
    target_compatible_with = ["@platforms//cpu:arm64"],
    visibility = ["//visibility:public"],
    deps = [
        "//aos:init",
        "//aos/events:shm_event_loop",
        "//aos/time",
        "//frc/can_logger:can_logger_lib",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
    ],
)
