package(default_visibility = ["//visibility:public"])
load("//aos/flatbuffers:generate.bzl", "static_flatbuffer")


static_flatbuffer(
    name = "superstructure_goal_fbs",
    srcs = [
        "superstructure_goal.fbs",
    ],
    deps = [
        "//frc/control_loops:control_loops_fbs",
        "//frc/control_loops:profiled_subsystem_fbs",
    ],
)

static_flatbuffer(
    name = "superstructure_output_fbs",
    srcs = [
        "superstructure_output.fbs",
    ],
)

static_flatbuffer(
    name = "superstructure_status_fbs",
    srcs = [
        "superstructure_status.fbs",
    ],
    deps = [
        "//frc/control_loops:control_loops_fbs",
        "//frc/control_loops:profiled_subsystem_fbs",
    ],
)

static_flatbuffer(
    name = "superstructure_position_fbs",
    srcs = [
        "superstructure_position.fbs",
    ],
    deps = [
        "//frc:can_configuration_fbs",
        "//frc/control_loops:can_talonfx_fbs",
        "//frc/control_loops:control_loops_fbs",
        "//frc/control_loops:profiled_subsystem_fbs",
    ],
)

cc_library(
    name = "superstructure_lib",
    srcs = [
        "superstructure.cc",
    ],
    hdrs = [
        "superstructure.h",
    ],
    data = [
    ],
    deps = [
        ":superstructure_goal_fbs",
        ":superstructure_output_fbs",
        ":superstructure_position_fbs",
        ":superstructure_status_fbs",
        "//aos:flatbuffer_merge",
        "//aos/events:event_loop",
        "//frc/constants:constants_sender_lib",
        "//frc/zeroing",
        "//frc/control_loops:static_zeroing_single_dof_profiled_subsystem",
        "//frc/control_loops:control_loop",
        "//frc/control_loops/drivetrain:drivetrain_can_position_fbs",
        "//frc/control_loops/drivetrain:drivetrain_status_fbs",
        "//frc/shooter_interpolation:interpolation",
        "//y2024_swerve:constants",
        "//y2024_swerve/constants:constants_fbs",
        "//y2024_swerve/constants:simulated_constants_sender",
    ],
)

cc_binary(
    name = "superstructure",
    srcs = [
        "superstructure_main.cc",
    ],
    deps = [
        ":superstructure_lib",
        "//aos:init",
        "//aos/events:shm_event_loop",
    ],
)