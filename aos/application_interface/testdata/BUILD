load("//aos:config.bzl", "aos_config")
load("//aos/flatbuffers:generate.bzl", "static_flatbuffer")

package(default_visibility = ["//aos/application_interface:__subpackages__"])

aos_config(
    name = "aos_config",
    src = "config.json",
    flatbuffers = [
        "//aos/events:event_loop_fbs",
        "//aos/logging:dynamic_log_command_fbs",
        "//aos/logging:log_message_fbs",
        ":message_fbs",
    ],
)

aos_config(
    name = "aos_multinode_config",
    src = "multinode_config.json",
    flatbuffers = [
        "//aos/events:event_loop_fbs",
        "//aos/logging:dynamic_log_command_fbs",
        "//aos/logging:log_message_fbs",
        "//aos/network:message_bridge_client_fbs",
        "//aos/network:remote_message_fbs",
        "//aos/network:timestamp_fbs",
        "//aos/network:message_bridge_server_fbs",
        ":message_fbs",
    ],
)

static_flatbuffer(
    name = "message_fbs",
    srcs = ["message.fbs"],
)
