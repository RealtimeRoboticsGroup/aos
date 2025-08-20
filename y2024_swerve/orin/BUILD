cc_binary(
    name = "canivore_logger",
    srcs = ["canivore_logger.cc"],
    target_compatible_with = ["@platforms//cpu:arm64"],
    visibility = ["//visibility:public"],
    deps = [
        "//aos:init",
        "//frc:can_configuration_fbs",
        "//frc/control_loops/swerve:swerve_drivetrain_can_position_fbs",
        "//frc/control_loops/swerve:swerve_drivetrain_output_fbs",
        "//frc/wpilib:can_sensor_reader",
        "//frc/wpilib:loop_output_handler",
        "//frc/wpilib:talonfx",
        "//y2024_swerve:constants",
    ],
)
