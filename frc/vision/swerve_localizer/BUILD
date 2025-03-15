load("//aos/flatbuffers:generate.bzl", "static_flatbuffer")

cc_library(
    name = "simulated_constants_sender_lib",
    hdrs = ["simulated_constants_sender_lib.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":field_map_constants_sender_lib",
        "//aos/events:simulated_event_loop",
        "//frc/vision:camera_constants_fbs",
        "//frc/vision:camera_constants_list_fbs",
        "//frc/vision:field_map_fbs",
    ],
)

cc_library(
    name = "field_map_constants_sender_lib",
    srcs = ["field_map_constants_sender_lib.cc"],
    hdrs = ["field_map_constants_sender_lib.h"],
    data = [
        "@frc2025_field_map_welded//file:frc2025r2.fmap",
    ],
    deps = [
        "//aos:json_to_flatbuffer",
        "//aos/events:event_loop",
        "//frc/vision:field_map_fbs",
        "//frc/vision:target_map_fbs",
        "@org_tuxfamily_eigen//:eigen",
    ],
)

cc_binary(
    name = "field_map_constants_sender",
    srcs = [
        "field_map_constants_sender.cc",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":field_map_constants_sender_lib",
        "//aos:configuration",
        "//aos:init",
        "//aos:json_to_flatbuffer",
        "//aos/events:shm_event_loop",
        "//frc/vision:field_map_fbs",
    ],
)

static_flatbuffer(
    name = "chassis_speeds_fbs",
    srcs = [
        "chassis_speeds.fbs",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
)

static_flatbuffer(
    name = "pose2d_fbs",
    srcs = [
        "pose2d.fbs",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
)

cc_binary(
    name = "network_tables_swerve_client",
    srcs = [
        "network_tables_swerve_client.cc",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":chassis_speeds_fbs",
        ":pose2d_fbs",
        "//aos:configuration",
        "//aos:init",
        "//aos:json_to_flatbuffer",
        "//aos/events:shm_event_loop",
        "//frc/input:joystick_state_fbs",
        "//frc/input:robot_state_fbs",
        "//frc/vision:field_map_fbs",
        "//frc/vision:target_map_fbs",
        "@com_github_wpilibsuite_allwpilib//ntcore:ntcore.static",
        "@com_github_wpilibsuite_allwpilib//wpimath:wpimath.static",
        "@com_google_absl//absl/log:die_if_null",
        "@org_tuxfamily_eigen//:eigen",
    ],
)
