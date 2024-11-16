cc_binary(
    name = "canivore_logger",
    srcs = ["canivore_logger.cc"],
    target_compatible_with = ["@platforms//cpu:arm64"],
    visibility = ["//visibility:public"],
    deps = [
        "//aos:init",
        "//frc971:can_configuration_fbs",
        "//frc971/control_loops/swerve:swerve_drivetrain_can_position_fbs",
        "//frc971/control_loops/swerve:swerve_drivetrain_output_fbs",
        "//frc971/wpilib:can_sensor_reader",
        "//frc971/wpilib:loop_output_handler",
        "//frc971/wpilib:talonfx",
        "//y2024_swerve:constants",
    ],
)
