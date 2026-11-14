load("@rules_cc//cc:cc_binary.bzl", "cc_binary")

cc_binary(
    name = "vision_constants_sender",
    srcs = [
        "vision_constants_sender.cc",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//aos:configuration",
        "//aos:init",
        "//aos/events:shm_event_loop",
        "//aos/util:file",
        "//frc/constants:constants_sender_lib",
        "//frc/vision:camera_constants_fbs",
        "//frc/vision:camera_constants_list_fbs",
        "@com_google_absl//absl/flags:flag",
        "@com_google_absl//absl/log:absl_log",
        "@com_google_absl//absl/strings",
    ],
)

cc_binary(
    name = "network_tables_publisher",
    srcs = ["network_tables_publisher.cc"],
    visibility = ["//visibility:public"],
    deps = [
        "//aos:configuration",
        "//aos:init",
        "//aos/events:shm_event_loop",
        "//frc/constants:constants_sender_lib",
        "//frc/control_loops/drivetrain/localization:localizer_output_fbs",
        "//frc/vision:camera_constants_fbs",
        "//frc/vision:field_map_fbs",
        "//frc/vision:target_map_fbs",
        "//frc/vision/field_map:status_fbs",
        "@com_github_wpilibsuite_allwpilib//ntcore:ntcore.static",
        "@com_github_wpilibsuite_allwpilib//wpimath:wpimath.static",
        "@com_google_absl//absl/flags:flag",
        "@eigen",
    ],
)

cc_binary(
    name = "network_tables_server",
    srcs = ["network_tables_server.cc"],
    visibility = ["//visibility:public"],
    deps = [
        "//aos:init",
        "@com_github_wpilibsuite_allwpilib//ntcore:ntcore.static",
        "@com_google_absl//absl/flags:flag",
    ],
)

cc_binary(
    name = "network_tables_client",
    srcs = ["network_tables_client.cc"],
    visibility = ["//visibility:public"],
    deps = [
        "//aos:init",
        "@com_github_wpilibsuite_allwpilib//ntcore:ntcore.static",
        "@com_github_wpilibsuite_allwpilib//wpimath:wpimath.static",
        "@com_google_absl//absl/flags:flag",
        "@com_google_absl//absl/log:absl_check",
        "@com_google_absl//absl/log:absl_log",
        "@com_google_absl//absl/log:die_if_null",
    ],
)

cc_binary(
    name = "network_tables_swerve_client",
    srcs = [
        "network_tables_swerve_client.cc",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//aos:configuration",
        "//aos:init",
        "//aos:json_to_flatbuffer",
        "//aos/events:shm_event_loop",
        "//aos/network:udp",
        "//frc/constants:constants_sender_lib",
        "//frc/control_loops/drivetrain/localization:localizer_output_fbs",
        "//frc/input:joystick_state_fbs",
        "//frc/input:robot_state_fbs",
        "//frc/vision:chassis_speeds_fbs",
        "//frc/vision:field_map_fbs",
        "//frc/vision:game_piece_locations_fbs",
        "//frc/vision:pose2d_fbs",
        "//frc/vision:target_map_fbs",
        "//frc/vision/field_map:udp_status_fbs",
        "@com_github_wpilibsuite_allwpilib//ntcore:ntcore.static",
        "@com_github_wpilibsuite_allwpilib//wpimath:wpimath.static",
        "@com_google_absl//absl/log:die_if_null",
        "@eigen",
    ],
)
