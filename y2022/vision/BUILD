load("//aos:config.bzl", "aos_config")

cc_binary(
    name = "camera_reader",
    srcs = [
        "camera_reader_main.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "//aos:init",
        "//aos/events:shm_event_loop",
        "//frc/vision:v4l2_reader",
    ],
)

aos_config(
    name = "camera_config",
    src = "single_node_camera_config.json",
    flatbuffers = [
        "//frc/vision:vision_fbs",
        "//aos/events:event_loop_fbs",
        "//aos/logging:log_message_fbs",
        "//aos/network:message_bridge_client_fbs",
        "//aos/network:message_bridge_server_fbs",
        "//aos/network:timestamp_fbs",
        "@com_github_foxglove_schemas//:schemas",
    ],
)
