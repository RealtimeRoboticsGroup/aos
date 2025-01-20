load("//aos/flatbuffers:generate.bzl", "static_flatbuffer")

static_flatbuffer(
    name = "constants_fbs",
    srcs = ["constants.fbs"],
    visibility = ["//visibility:public"],
    deps = [
        "//frc/control_loops:profiled_subsystem_fbs",
        "//frc/control_loops/drivetrain:drivetrain_config_fbs",
        "//frc/vision:calibration_fbs",
        "//frc/vision:target_map_fbs",
        "//frc/zeroing:constants_fbs",
    ],
)
