load("//aos/flatbuffers:generate.bzl", "static_flatbuffer")

static_flatbuffer(
    name = "constants_fbs",
    srcs = ["constants.fbs"],
    visibility = ["//visibility:public"],
    deps = [
        "//frc971/control_loops:profiled_subsystem_fbs",
        "//frc971/control_loops/drivetrain:drivetrain_config_fbs",
        "//frc971/vision:calibration_fbs",
        "//frc971/vision:target_map_fbs",
        "//frc971/zeroing:constants_fbs",
    ],
)
