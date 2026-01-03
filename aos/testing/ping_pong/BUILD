load("@com_github_google_flatbuffers//:typescript.bzl", "flatbuffer_ts_library")
load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")
load("//aos:config.bzl", "aos_config")
load("//aos/flatbuffers:generate.bzl", "static_flatbuffer")
load("//tools/rust:defs.bzl", "flatbuffer_rust_library", "rust_library", "rust_test")

package(default_visibility = ["//visibility:public"])

static_flatbuffer(
    name = "ping_fbs",
    srcs = ["ping.fbs"],
)

flatbuffer_ts_library(
    name = "ping_ts_fbs",
    srcs = ["ping.fbs"],
    ts_files = [
        "aos/examples/ping.ts",
        "aos/examples.ts",
    ],
)

flatbuffer_rust_library(
    name = "ping_rust_fbs",
    srcs = ["ping.fbs"],
)

flatbuffer_rust_library(
    name = "pong_rust_fbs",
    srcs = ["pong.fbs"],
)

static_flatbuffer(
    name = "pong_fbs",
    srcs = ["pong.fbs"],
)

cc_library(
    name = "ping_lib",
    srcs = [
        "ping_lib.cc",
    ],
    hdrs = [
        "ping_lib.h",
    ],
    deps = [
        ":ping_fbs",
        ":pong_fbs",
        "//aos:json_to_flatbuffer",
        "//aos/events:event_loop",
        "//aos/logging",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
    ],
)

rust_test(
    name = "pingpong_test_rs",
    srcs = [
        "pingpong_test.rs",
    ],
    data = [":pingpong_config"],
    # TODO(adam.snaider): Remove later. For now we need this because when
    # a rust test crashes inside of C++, the output gets captured which makes
    # it pretty much impossible to figure out what happened.
    env = {"RUST_TEST_NOCAPTURE": "1"},
    deps = [
        ":ping_lib_rs",
        ":ping_rust_fbs",
        ":pong_lib_rs",
        ":pong_rust_fbs",
        "//aos/testing:aos_rs",
        "@com_github_google_flatbuffers//rust",
        "@crate_index//:futures",
    ],
)

rust_library(
    name = "ping_lib_rs",
    srcs = [
        "ping_lib.rs",
    ],
    crate_name = "ping_lib",
    deps = [
        ":ping_rust_fbs",
        ":pong_rust_fbs",
        "//aos:aos_rs",
        "@com_github_google_flatbuffers//rust",
        "@crate_index//:futures",
        "@crate_index//:log",
    ],
)

rust_library(
    name = "pong_lib_rs",
    srcs = [
        "pong_lib.rs",
    ],
    crate_name = "pong_lib",
    deps = [
        ":ping_rust_fbs",
        ":pong_rust_fbs",
        "//aos:aos_rs",
        "@com_github_google_flatbuffers//rust",
        "@crate_index//:futures",
        "@crate_index//:log",
    ],
)

aos_config(
    name = "pingpong_config",
    src = "pingpong.json",
    flatbuffers = [
        ":ping_fbs",
        ":pong_fbs",
    ],
    deps = [":aos_config"],
)

# TODO(ben): I'm not I follow what is going on with these configs.
aos_config(
    name = "aos_config",
    src = "aos.json",
    flatbuffers = [
        "//aos/events:event_loop_fbs",
        "//aos/logging:dynamic_log_command_fbs",
        "//aos/logging:log_message_fbs",
        "//aos/starter:starter_fbs",
        "//aos/starter:starter_rpc_fbs",
    ],
)

[
    aos_config(
        name = config + "_config",
        src = config + ".json",
        flatbuffers = [
            ":ping_fbs",
            ":pong_fbs",
            "//aos/network:message_bridge_client_fbs",
            "//aos/network:remote_message_fbs",
            "//aos/network:timestamp_fbs",
            "//aos/network:message_bridge_server_fbs",
        ],
        deps = [":aos_config"],
    )
    for config in [
        "multinode_pingpong_test_split",
        "multinode_pingpong",
        "multinode_pingpong_test_combined",
    ]
]

cc_library(
    name = "pong_lib",
    srcs = [
        "pong_lib.cc",
    ],
    hdrs = [
        "pong_lib.h",
    ],
    deps = [
        ":ping_fbs",
        ":pong_fbs",
        "//aos/events:event_loop",
        "//aos/logging",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
    ],
)

cc_test(
    name = "pingpong_test",
    srcs = ["pingpong_test.cc"],
    data = [
        ":multinode_pingpong_test_split_config",
        ":pingpong_config",
    ],
    deps = [
        ":ping_lib",
        ":pong_lib",
        "//aos:configuration",
        "//aos:flatbuffers",
        "//aos/events:simulated_event_loop",
        "//aos/testing:googletest",
        "//aos/testing:path",
    ],
)
