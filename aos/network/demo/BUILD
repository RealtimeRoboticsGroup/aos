load("@bazel_skylib//rules:common_settings.bzl", "int_flag", "string_flag")
load("@rules_cc//cc:cc_binary.bzl", "cc_binary")
load("//aos:config.bzl", "aos_config")
load("//aos/flatbuffers:generate.bzl", "static_flatbuffer")
load("//tools/build_rules:template.bzl", "jinja2_template")
load("//tools/python:defs.bzl", "py_library")
load(":demo.bzl", "py_demo_runner_binary")

string_flag(
    name = "ping_node_name",
    build_setting_default = "ping_node",
)

string_flag(
    name = "pong_node_name",
    build_setting_default = "pong_node",
)

int_flag(
    name = "variable_sleep_us",
    build_setting_default = 10 * 1000,
)

int_flag(
    name = "variable_message_size",
    build_setting_default = 10 * 1000,
)

int_flag(
    name = "variable_num_messages",
    build_setting_default = 10,
)

int_flag(
    name = "ping_priority",
    build_setting_default = 10,
)

int_flag(
    name = "variable_size_message_priority",
    build_setting_default = 10,
)

PING_TARGETS = [
    "//documentation/examples/ping_pong:ping",
]

PONG_TARGETS = [
    "//documentation/examples/ping_pong:pong",
]

jinja2_template(
    name = "hostnames.py",
    src = "hostnames.py.jinja",
    flag_parameters = {
        "PING_NODE_NAME": ":ping_node_name",
        "PONG_NODE_NAME": ":pong_node_name",
    },
)

py_library(
    name = "hostnames",
    srcs = ["hostnames.py"],
)

py_demo_runner_binary(
    name = "run_ping",
    args = [
        "ping",
    ],
    data = PING_TARGETS,
    deps = [
        ":hostnames",
    ],
)

py_demo_runner_binary(
    name = "run_pong",
    args = [
        "pong",
    ],
    data = PONG_TARGETS,
    deps = [
        ":hostnames",
    ],
)

jinja2_template(
    name = "aos_demo_config_source.json",
    src = "aos_demo_config.json.jinja",
    flag_parameters = {
        "PING_NODE_NAME": ":ping_node_name",
        "PING_PRIORITY": ":ping_priority",
        "PONG_NODE_NAME": ":pong_node_name",
        "VARIABLE_MESSAGE_SIZE": ":variable_message_size",
        "VARIABLE_NUM_MESSAGES": ":variable_num_messages",
        "VARIABLE_SIZE_MESSAGE_PRIORITY": ":variable_size_message_priority",
        "VARIABLE_SLEEP_US": ":variable_sleep_us",
    },
)

aos_config(
    name = "aos_demo_config",
    src = ":aos_demo_config_source.json",
    flatbuffers = [
        ":variable_size_message_fbs",
        "//aos/events:event_loop_fbs",
        "//aos/testing/ping_pong:ping_fbs",
        "//aos/testing/ping_pong:pong_fbs",
        "//aos/logging:dynamic_log_command_fbs",
        "//aos/logging:log_message_fbs",
        "//aos/network:message_bridge_client_fbs",
        "//aos/network:remote_message_fbs",
        "//aos/network:timestamp_fbs",
        "//aos/network:message_bridge_server_fbs",
        "//aos/starter:starter_fbs",
        "//aos/starter:starter_rpc_fbs",
    ],
)

static_flatbuffer(
    name = "variable_size_message_fbs",
    srcs = ["variable_size_message.fbs"],
)

cc_binary(
    name = "variable_size_message_sender",
    srcs = [
        "variable_size_message_sender.cc",
    ],
    deps = [
        ":variable_size_message_fbs",
        "//aos:configuration",
        "//aos:init",
        "//aos/events:shm_event_loop",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:absl_check",
        "@com_google_absl//absl/log:die_if_null",
    ],
)
