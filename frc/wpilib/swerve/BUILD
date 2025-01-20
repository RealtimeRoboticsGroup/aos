load("//aos/flatbuffers:generate.bzl", "static_flatbuffer")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "swerve_drivetrain_writer",
    srcs = [
        "swerve_drivetrain_writer.cc",
    ],
    hdrs = [
        "swerve_drivetrain_writer.h",
    ],
    deps = [
        ":swerve_module",
        "//aos/logging",
        "//frc:can_configuration_fbs",
        "//frc/control_loops/swerve:swerve_drivetrain_can_position_fbs",
        "//frc/control_loops/swerve:swerve_drivetrain_output_fbs",
        "//frc/control_loops/swerve:swerve_drivetrain_position_fbs",
        "//frc/wpilib:loop_output_handler",
        "//frc/wpilib:talonfx",
        "//third_party:phoenix6",
    ],
)

cc_library(
    name = "swerve_module",
    hdrs = [
        "swerve_module.h",
    ],
    deps = [
        ":swerve_constants_fbs",
        "//frc/wpilib:encoder_and_potentiometer",
        "//frc/wpilib:talonfx",
    ],
)

static_flatbuffer(
    name = "swerve_constants_fbs",
    srcs = ["swerve_constants.fbs"],
    visibility = ["//visibility:public"],
)
