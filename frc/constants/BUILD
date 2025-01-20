cc_library(
    name = "constants_sender_lib",
    hdrs = [
        "constants_sender_lib.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "//aos:flatbuffer_merge",
        "//aos:json_to_flatbuffer",
        "//aos/events:event_loop",
        "//aos/events:shm_event_loop",
        "//aos/network:team_number",
        "@com_google_absl//absl/flags:flag",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
    ],
)

cc_binary(
    name = "constants_sender_example",
    srcs = [
        "constants_sender_example.cc",
    ],
    data = [
        "//frc/constants/testdata:aos_config",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "//aos:configuration",
        "//aos:init",
        "//aos:json_to_flatbuffer",
        "//aos/events:shm_event_loop",
        "//frc/constants:constants_sender_lib",
        "//frc/constants/testdata:constants_data_fbs",
        "//frc/constants/testdata:constants_list_fbs",
        "@com_google_absl//absl/flags:flag",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
    ],
)

cc_test(
    name = "constants_sender_test",
    srcs = [
        "constants_sender_test.cc",
    ],
    data = [
        "//frc/constants/testdata:aos_config",
        "//frc/constants/testdata:syntax_error.json",
        "//frc/constants/testdata:test_constants.json",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":constants_sender_lib",
        "//aos:configuration",
        "//aos:flatbuffer_merge",
        "//aos:flatbuffers",
        "//aos:json_to_flatbuffer",
        "//aos/events:event_loop",
        "//aos/events:simulated_event_loop",
        "//aos/testing:googletest",
        "//aos/testing:path",
        "//frc/constants/testdata:constants_data_fbs",
        "//frc/constants/testdata:constants_list_fbs",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
    ],
)
