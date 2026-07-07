load("@aos//aos/flatbuffers:build_defs.bzl", "flatbuffer_cc_library")
load("@aos//aos/flatbuffers:typescript.bzl", "flatbuffer_ts_library")
load("@com_github_grpc_grpc//bazel:cc_grpc_library.bzl", "cc_grpc_library")
load("@com_google_protobuf//bazel:cc_proto_library.bzl", "cc_proto_library")
load("@com_google_protobuf//bazel:proto_library.bzl", "proto_library")
load("@rules_cc//cc:cc_binary.bzl", "cc_binary")
load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")
load("//aos:config.bzl", "aos_config")
load("//aos:flatbuffers.bzl", "cc_static_flatbuffer")
load("//aos/flatbuffers:generate.bzl", "static_flatbuffer")
load("//aos/seasocks:gen_embedded.bzl", "gen_embedded")

package(
    default_applicable_licenses = ["//:license"],
    default_visibility = ["//visibility:public"],
)

static_flatbuffer(
    name = "remote_message_fbs",
    srcs = ["remote_message.fbs"],
)

cc_static_flatbuffer(
    name = "remote_message_schema",
    function = "aos::message_bridge::RemoteMessageSchema",
    target = ":remote_message_fbs_reflection_out",
)

static_flatbuffer(
    name = "connect_fbs",
    srcs = ["connect.fbs"],
    deps = [
        "//aos:configuration_fbs",
    ],
)

flatbuffer_ts_library(
    name = "connect_ts_fbs",
    srcs = ["connect.fbs"],
    include_reflection = True,
    ts_files = [
        "aos/message-bridge/connect.ts",
        "aos/channel.ts",
        "aos/logger-config.ts",
        "aos/connection.ts",
        "aos/read-method.ts",
        "aos/node.ts",
    ],
    deps = [
        "//aos:configuration_ts_fbs",
    ],
)

static_flatbuffer(
    name = "remote_data_fbs",
    srcs = ["remote_data.fbs"],
)

static_flatbuffer(
    name = "timestamp_fbs",
    srcs = ["timestamp.fbs"],
    deps = [
        "//aos:configuration_fbs",
    ],
)

cc_static_flatbuffer(
    name = "timestamp_schema",
    function = "aos::message_bridge::TimestampSchema",
    target = ":timestamp_fbs_reflection_out",
    visibility = ["//visibility:public"],
)

static_flatbuffer(
    name = "message_bridge_client_fbs",
    srcs = ["message_bridge_client.fbs"],
    deps = [
        ":message_bridge_server_fbs",
        "//aos:configuration_fbs",
    ],
)

flatbuffer_ts_library(
    name = "message_bridge_client_ts_fbs",
    srcs = ["message_bridge_client.fbs"],
    ts_files = [
        "aos/message-bridge.ts",
        "aos/message-bridge/client-connection.ts",
        "aos/message-bridge/client-statistics.ts",
        "aos/message-bridge/server-connection.ts",
        "aos/message-bridge/server-statistics.ts",
        "aos/message-bridge/server-channel-statistics.ts",
        "aos/message-bridge/state.ts",
        "aos/node.ts",
    ],
    deps = [
        ":message_bridge_server_ts_fbs",
    ],
)

cc_static_flatbuffer(
    name = "message_bridge_client_schema",
    function = "aos::message_bridge::ClientStatisticsSchema",
    target = ":message_bridge_client_fbs_reflection_out",
    visibility = ["//visibility:public"],
)

static_flatbuffer(
    name = "sctp_config_fbs",
    srcs = ["sctp_config.fbs"],
)

cc_static_flatbuffer(
    name = "sctp_config_schema",
    function = "aos::message_bridge::SctpConfig",
    target = ":sctp_config_fbs_reflection_out",
)

static_flatbuffer(
    name = "sctp_config_request_fbs",
    srcs = ["sctp_config_request.fbs"],
)

cc_static_flatbuffer(
    name = "sctp_config_request_schema",
    function = "aos::message_bridge::SctpConfigRequest",
    target = ":sctp_config_request_fbs_reflection_out",
    visibility = ["//visibility:public"],
)

static_flatbuffer(
    name = "message_bridge_server_fbs",
    srcs = ["message_bridge_server.fbs"],
    deps = [
        "//aos:configuration_fbs",
    ],
)

flatbuffer_ts_library(
    name = "message_bridge_server_ts_fbs",
    srcs = ["message_bridge_server.fbs"],
    ts_files = [
        "aos/message-bridge/server-connection.ts",
        "aos/message-bridge/server-statistics.ts",
        "aos/message-bridge/server-channel-statistics.ts",
        "aos/message-bridge/state.ts",
        "aos/message-bridge.ts",
        "aos/node.ts",
    ],
    deps = [
        "//aos:configuration_ts_fbs",
    ],
)

cc_static_flatbuffer(
    name = "message_bridge_server_schema",
    function = "aos::message_bridge::ServerStatisticsSchema",
    target = ":message_bridge_server_fbs_reflection_out",
    visibility = ["//visibility:public"],
)

proto_library(
    name = "message_bridge_auth_proto",
    srcs = [
        "message_bridge_auth.proto",
    ],
    visibility = [":__pkg__"],
)

cc_proto_library(
    name = "message_bridge_auth_proto_cc",
    visibility = ["//visibility:public"],
    deps = [
        ":message_bridge_auth_proto",
    ],
)

cc_grpc_library(
    name = "message_bridge_auth_proto_grpc_cc",
    srcs = [
        ":message_bridge_auth_proto",
    ],
    grpc_only = True,
    deps = [
        ":message_bridge_auth_proto_cc",
    ],
)

cc_library(
    name = "team_number",
    srcs = [
        "team_number.cc",
    ],
    hdrs = [
        "team_number.h",
    ],
    deps = [
        "@com_google_absl//absl/flags:flag",
        "@com_google_absl//absl/log:absl_check",
        "@com_google_absl//absl/log:absl_log",
        "@com_google_absl//absl/strings",
    ],
)

cc_test(
    name = "team_number_test",
    srcs = [
        "team_number_test.cc",
    ],
    deps = [
        ":team_number",
        "//aos:configuration",
        "//aos/testing:googletest",
    ],
)

cc_library(
    name = "sctp_error",
    srcs = [
        "sctp_error.cc",
    ],
    hdrs = [
        "sctp_error.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
)

cc_test(
    name = "sctp_error_test",
    srcs = [
        "sctp_error_test.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":sctp_error",
        "//aos/testing:googletest",
    ],
)

cc_library(
    name = "sctp_lib",
    srcs = [
        "sctp_lib.cc",
    ],
    hdrs = [
        "sctp_lib.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "//aos:realtime",
        "//aos:unique_malloc_ptr",
        "//aos/util:file",
        "@com_google_absl//absl/flags:flag",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/types:span",
    ],
)

cc_test(
    name = "sctp_test",
    srcs = [
        "sctp_test.cc",
    ],
    tags = [
        # We really don't want networking enabled for this, or the test fails.
        "block-network",
        # Fakeroot is required to enable "net.sctp.auth_enable".
        "requires-fakeroot",
        "no-remote-exec",
    ],
    target_compatible_with = ["@platforms//cpu:x86_64"],
    deps = [
        ":sctp_client",
        ":sctp_lib",
        ":sctp_server",
        "//aos/events:epoll",
        "//aos/testing:googletest",
        "//aos/util:file",
    ],
)

cc_library(
    name = "sctp_server",
    srcs = [
        "sctp_server.cc",
    ],
    hdrs = [
        "sctp_server.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":sctp_lib",
    ],
)

cc_library(
    name = "message_bridge_protocol",
    srcs = [
        "message_bridge_protocol.cc",
    ],
    hdrs = [
        "message_bridge_protocol.h",
    ],
    deps = [
        ":connect_fbs",
        "//aos:configuration",
        "//aos:flatbuffer_merge",
        "//aos:flatbuffers",
        "//aos:uuid",
        "@com_github_google_flatbuffers//:flatbuffers",
    ],
)

cc_library(
    name = "message_bridge_server_status",
    srcs = [
        "message_bridge_server_status.cc",
    ],
    hdrs = [
        "message_bridge_server_status.h",
    ],
    deps = [
        ":message_bridge_client_fbs",
        ":message_bridge_server_fbs",
        ":timestamp_fbs",
        ":timestamp_filter",
        "//aos:flatbuffer_merge",
        "//aos:flatbuffers",
        "//aos/events:event_loop",
        "//aos/time",
    ],
)

cc_library(
    name = "timestamp_channel",
    srcs = ["timestamp_channel.cc"],
    hdrs = ["timestamp_channel.h"],
    deps = [
        ":remote_message_fbs",
        "//aos:configuration",
        "//aos/events:event_loop",
        "@com_google_absl//absl/container:btree",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/strings",
    ],
)

cc_library(
    name = "message_bridge_server_lib",
    srcs = [
        "message_bridge_server_lib.cc",
    ],
    hdrs = [
        "message_bridge_server_lib.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":connect_fbs",
        ":message_bridge_client_fbs",
        ":message_bridge_protocol",
        ":message_bridge_server_fbs",
        ":message_bridge_server_status",
        ":remote_data_fbs",
        ":remote_message_fbs",
        ":sctp_config_fbs",
        ":sctp_config_request_fbs",
        ":sctp_lib",
        ":sctp_server",
        ":timestamp_channel",
        ":timestamp_fbs",
        "//aos:ftrace",
        "//aos:unique_malloc_ptr",
        "//aos/events:shm_event_loop",
        "//aos/events/logging:log_reader",
    ],
)

cc_binary(
    name = "message_bridge_server",
    srcs = [
        "message_bridge_server.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":message_bridge_server_lib",
        ":sctp_lib",
        "//aos:init",
        "//aos:json_to_flatbuffer",
        "//aos:sha256",
        "//aos/events:shm_event_loop",
        "//aos/logging:dynamic_logging",
    ],
)

cc_library(
    name = "sctp_client",
    srcs = [
        "sctp_client.cc",
    ],
    hdrs = [
        "sctp_client.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":sctp_lib",
        "@com_google_absl//absl/flags:flag",
    ],
)

cc_library(
    name = "message_bridge_client_status",
    srcs = [
        "message_bridge_client_status.cc",
    ],
    hdrs = [
        "message_bridge_client_status.h",
    ],
    deps = [
        ":message_bridge_client_fbs",
        ":message_bridge_server_fbs",
        ":timestamp_filter",
        "//aos:flatbuffers",
        "//aos/events:event_loop",
        "//aos/time",
    ],
)

cc_library(
    name = "message_bridge_client_lib",
    srcs = [
        "message_bridge_client_lib.cc",
    ],
    hdrs = [
        "message_bridge_client_lib.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":connect_fbs",
        ":message_bridge_client_fbs",
        ":message_bridge_client_status",
        ":message_bridge_protocol",
        ":message_bridge_server_fbs",
        ":remote_data_fbs",
        ":remote_message_fbs",
        ":sctp_client",
        ":sctp_config_fbs",
        ":sctp_config_request_fbs",
        ":sctp_error",
        ":timestamp_fbs",
        "//aos:ftrace",
        "//aos/containers:ring_buffer",
        "//aos/events:shm_event_loop",
        "//aos/events/logging:log_reader",
    ],
)

cc_binary(
    name = "message_bridge_client",
    srcs = [
        "message_bridge_client.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":message_bridge_client_lib",
        ":sctp_lib",
        "//aos:init",
        "//aos:json_to_flatbuffer",
        "//aos:sha256",
        "//aos/events:shm_event_loop",
        "//aos/logging:dynamic_logging",
        "//aos/util:file",
    ],
)

cc_library(
    name = "message_bridge_auth_server_lib",
    srcs = [
        "message_bridge_auth_server_lib.cc",
    ],
    hdrs = [
        "message_bridge_auth_server_lib.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":message_bridge_auth_proto_cc",
        ":message_bridge_auth_proto_grpc_cc",
        "//aos/logging",
        "//aos/util:file",
        "@com_github_grpc_grpc//:grpc++",
        "@com_google_absl//absl/log:absl_check",
        "@com_google_absl//absl/log:absl_log",
        "@com_google_absl//absl/strings",
    ],
)

cc_library(
    name = "message_bridge_auth_client_lib",
    srcs = [
        "message_bridge_auth_client_lib.cc",
    ],
    hdrs = [
        "message_bridge_auth_client_lib.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":message_bridge_auth_proto_grpc_cc",
        ":message_bridge_client_fbs",
        ":message_bridge_server_fbs",
        ":sctp_config_fbs",
        ":sctp_config_request_fbs",
        "//aos/events:event_loop",
        "//aos/logging",
        "//aos/util:file",
        "@com_github_grpc_grpc//:grpc++",
        "@com_google_absl//absl/strings",
    ],
)

cc_test(
    name = "message_bridge_auth_test",
    srcs = [
        "message_bridge_auth_test.cc",
    ],
    data = [
        ":message_bridge_auth_test_config",
    ],
    tags = ["requires-network"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":message_bridge_auth_client_lib",
        ":message_bridge_auth_server_lib",
        "//aos:configuration",
        "//aos/events:simulated_event_loop",
        "//aos/logging",
        "//aos/testing:googletest",
        "//aos/testing:path",
        "//aos/util:file",
        "@com_github_grpc_grpc//:grpc++",
        "@com_google_absl//absl/strings",
    ],
)

aos_config(
    name = "message_bridge_test_combined_timestamps_common_config",
    src = "message_bridge_test_combined_timestamps_common.json",
    flatbuffers = [
        ":remote_message_fbs",
        ":sctp_config_fbs",
        ":sctp_config_request_fbs",
        "//aos/testing/ping_pong:ping_fbs",
        "//aos/testing/ping_pong:pong_fbs",
        "//aos/network:message_bridge_client_fbs",
        "//aos/network:message_bridge_server_fbs",
        "//aos/network:timestamp_fbs",
    ],
    deps = ["//aos/events:aos_config"],
)

aos_config(
    name = "message_bridge_test_mismatched_configs_pi1_and_pi2_config",
    src = "message_bridge_test_mismatched_configs_pi1_and_pi2.json",
    flatbuffers = [
        ":remote_message_fbs",
        "//aos/network:message_bridge_client_fbs",
        "//aos/network:message_bridge_server_fbs",
        "//aos/network:timestamp_fbs",
    ],
    deps = ["//aos/events:aos_config"],
)

aos_config(
    name = "message_bridge_test_mismatched_configs_pi1_and_pi3_config",
    src = "message_bridge_test_mismatched_configs_pi1_and_pi3.json",
    flatbuffers = [
        ":remote_message_fbs",
        "//aos/network:message_bridge_client_fbs",
        "//aos/network:message_bridge_server_fbs",
        "//aos/network:timestamp_fbs",
    ],
    deps = ["//aos/events:aos_config"],
)

aos_config(
    name = "message_bridge_auth_test_config",
    src = "message_bridge_auth_test.json",
    flatbuffers = [
        ":sctp_config_fbs",
        ":sctp_config_request_fbs",
    ],
    deps = ["//aos/events:aos_config"],
)

aos_config(
    name = "message_bridge_test_common_config",
    src = "message_bridge_test_common.json",
    flatbuffers = [
        ":remote_message_fbs",
        ":sctp_config_fbs",
        ":sctp_config_request_fbs",
        "//aos/testing/ping_pong:ping_fbs",
        "//aos/testing/ping_pong:pong_fbs",
        "//aos/network:message_bridge_client_fbs",
        "//aos/network:message_bridge_server_fbs",
        "//aos/network:timestamp_fbs",
    ],
    deps = ["//aos/events:aos_config"],
)

aos_config(
    name = "timestamp_channel_test_config",
    src = "timestamp_channel_test.json",
    flatbuffers = [
        ":remote_message_fbs",
        "//aos/testing/ping_pong:ping_fbs",
        "//aos/testing/ping_pong:pong_fbs",
        "//aos/network:message_bridge_client_fbs",
        "//aos/network:message_bridge_server_fbs",
        "//aos/network:timestamp_fbs",
    ],
    deps = ["//aos/events:aos_config"],
)

cc_library(
    name = "message_bridge_test_lib",
    testonly = True,
    srcs = ["message_bridge_test_lib.cc"],
    hdrs = ["message_bridge_test_lib.h"],
    deps = [
        ":message_bridge_client_lib",
        ":message_bridge_server_lib",
        "//aos:json_to_flatbuffer",
        "//aos:sha256",
        "//aos/events:shm_event_loop",
        "//aos/ipc_lib:event",
        "//aos/testing:googletest",
        "//aos/testing:path",
        "//aos/testing/ping_pong:ping_fbs",
        "//aos/testing/ping_pong:pong_fbs",
        "//aos/util:application_name",
    ],
)

cc_test(
    name = "message_bridge_retry_test",
    srcs = [
        "message_bridge_retry_test.cc",
    ],
    data = [
        ":message_bridge_test_common_config",
    ],
    # Somewhat flaky due to relying on the behavior & timing of the host system's network stack.
    flaky = True,
    tags = [
        "block-network",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":message_bridge_server_lib",
        ":message_bridge_test_lib",
        "//aos:json_to_flatbuffer",
        "//aos:sha256",
        "//aos/events:shm_event_loop",
        "//aos/ipc_lib:event",
        "//aos/testing:googletest",
        "//aos/testing:path",
        "//aos/testing/ping_pong:ping_fbs",
        "//aos/testing/ping_pong:pong_fbs",
        "//aos/util:application_name",
    ],
)

cc_test(
    name = "message_bridge_test",
    srcs = [
        "message_bridge_test.cc",
    ],
    data = [
        ":message_bridge_test_combined_timestamps_common_config",
        ":message_bridge_test_common_config",
        ":message_bridge_test_mismatched_configs_pi1_and_pi2_config",
        ":message_bridge_test_mismatched_configs_pi1_and_pi3_config",
    ],
    flaky = True,
    shard_count = 16,
    tags = [
        "block-network",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":message_bridge_client_lib",
        ":message_bridge_server_lib",
        ":message_bridge_test_lib",
        "//aos:json_to_flatbuffer",
        "//aos:sha256",
        "//aos/events:shm_event_loop",
        "//aos/ipc_lib:event",
        "//aos/testing:googletest",
        "//aos/testing:path",
        "//aos/testing/ping_pong:ping_fbs",
        "//aos/testing/ping_pong:pong_fbs",
        "//aos/util:application_name",
    ],
)

cc_test(
    name = "message_bridge_server_status_test",
    srcs = [
        "message_bridge_server_status_test.cc",
    ],
    data = [
        ":message_bridge_test_combined_timestamps_common_config",
    ],
    deps = [
        ":message_bridge_server_status",
        "//aos/events:simulated_event_loop",
        "//aos/testing:googletest",
        "//aos/testing:path",
    ],
)

flatbuffer_cc_library(
    name = "web_proxy_fbs",
    srcs = ["web_proxy.fbs"],
    gen_reflections = True,
    deps = [
        ":connect_fbs_fbs",
        "//aos:configuration_fbs_fbs",
    ],
)

flatbuffer_ts_library(
    name = "web_proxy_ts_fbs",
    srcs = ["web_proxy.fbs"],
    include_reflection = True,
    ts_files = [
        "aos/web-proxy/sdp-type.ts",
        "aos/web-proxy/web-socket-sdp.ts",
        "aos/web-proxy/web-socket-ice.ts",
        "aos/web-proxy/web-socket-message.ts",
        "aos/web-proxy/message-header.ts",
        "aos/web-proxy/payload.ts",
        "aos/web-proxy/transfer-method.ts",
        "aos/web-proxy/channel-request.ts",
        "aos/web-proxy/channel-state.ts",
        "aos/web-proxy/subscriber-request.ts",
        "aos/web-proxy.ts",
        "aos/message-bridge/connect.ts",
        "aos/channel.ts",
        "aos/logger-config.ts",
        "aos/connection.ts",
        "aos/read-method.ts",
        "aos/node.ts",
    ],
    deps = [
        ":connect_ts_fbs",
        "//aos:configuration_ts_fbs",
    ],
)

cc_library(
    name = "web_proxy_utils",
    srcs = ["web_proxy_utils.cc"],
    hdrs = ["web_proxy_utils.h"],
    deps = [
        ":connect_fbs",
        ":web_proxy_fbs",
        "//aos:configuration_fbs",
        "//aos/events:event_loop",
        "@com_github_google_flatbuffers//:flatbuffers",
    ],
)

cc_library(
    name = "web_proxy",
    srcs = [
        "rawrtc.cc",
        "web_proxy.cc",
    ],
    hdrs = [
        "rawrtc.h",
        "web_proxy.h",
    ],
    deps = [
        ":connect_fbs",
        ":gen_embedded",
        ":web_proxy_fbs",
        ":web_proxy_utils",
        "//aos/events:shm_event_loop",
        "//aos/mutex",
        "//aos/seasocks:seasocks_logger",
        "//third_party/seasocks",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
        "@rawrtc",
    ],
)

gen_embedded(
    name = "gen_embedded",
    srcs = glob(
        include = ["www_defaults/**/*"],
        exclude = ["www/**/*"],
    ),
    visibility = ["//visibility:public"],
)

cc_binary(
    name = "web_proxy_main",
    srcs = ["web_proxy_main.cc"],
    data = [
        "//aos/network/www:files",
        "//aos/network/www:main_bundle.min.js",
    ],
    deps = [
        ":web_proxy",
        "//aos:init",
        "//aos/events:shm_event_loop",
        "@com_github_google_flatbuffers//:flatbuffers",
    ],
)

cc_binary(
    name = "log_web_proxy_main",
    srcs = ["log_web_proxy_main.cc"],
    args = [
        "--data_dir=aos/network/www",
    ],
    deps = [
        ":web_proxy",
        "//aos:init",
        "//aos/events:simulated_event_loop",
        "//aos/events/logging:log_reader",
        "@com_github_google_flatbuffers//:flatbuffers",
    ],
)

cc_library(
    name = "timestamp_filter",
    srcs = [
        "timestamp_filter.cc",
    ],
    hdrs = [
        "timestamp_filter.h",
    ],
    deps = [
        ":noncausal_timestamp_filter",
        "//aos:configuration",
        "//aos/events/logging:boot_timestamp",
        "//aos/time",
        "@com_google_absl//absl/numeric:int128",
        "@com_google_absl//absl/strings",
    ],
)

cc_library(
    name = "noncausal_timestamp_filter",
    srcs = [
        "noncausal_timestamp_filter.cc",
    ],
    hdrs = [
        "noncausal_timestamp_filter.h",
    ],
    deps = [
        "//aos:configuration",
        "//aos/events/logging:boot_timestamp",
        "//aos/time",
        "@com_google_absl//absl/numeric:int128",
        "@com_google_absl//absl/strings",
    ],
)

cc_library(
    name = "multinode_timestamp_filter",
    srcs = ["multinode_timestamp_filter.cc"],
    hdrs = ["multinode_timestamp_filter.h"],
    deps = [
        ":noncausal_timestamp_filter",
        "//aos:configuration",
        "//aos/events:simulated_event_loop",
        "//aos/events/logging:boot_timestamp",
        "//aos/events/logging:logfile_utils",
        "//aos/time",
        "@eigen",
    ],
)

cc_test(
    name = "timestamp_filter_test",
    srcs = [
        "timestamp_filter_test.cc",
    ],
    deps = [
        ":timestamp_filter",
        "//aos:json_to_flatbuffer",
        "//aos/testing:googletest",
    ],
)

cc_test(
    name = "noncausal_timestamp_filter_test",
    srcs = [
        "noncausal_timestamp_filter_test.cc",
    ],
    deps = [
        ":noncausal_timestamp_filter",
        "//aos:json_to_flatbuffer",
        "//aos/testing:googletest",
    ],
)

cc_library(
    name = "testing_time_converter",
    testonly = True,
    srcs = ["testing_time_converter.cc"],
    hdrs = ["testing_time_converter.h"],
    deps = [
        ":multinode_timestamp_filter",
        "//aos/events:simulated_event_loop",
        "//aos/events/logging:boot_timestamp",
        "//aos/time",
    ],
)

cc_binary(
    name = "sctp_lib_shim.so",
    testonly = True,
    srcs = [
        "sctp_lib_shim.c",
    ],
    linkopts = [
        "-ldl",
    ],
    linkshared = True,
    target_compatible_with = ["@platforms//os:linux"],
)

cc_binary(
    name = "sctp_lib_test_binary",
    testonly = True,
    srcs = [
        "sctp_lib_test.cc",
    ],
    linkstatic = False,
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":sctp_lib",
        "//aos:init",
    ],
)

sh_test(
    name = "sctp_lib_test",
    srcs = [
        "sctp_lib_test.sh",
    ],
    args = [
        "$(location :sctp_lib_test_binary)",
        "$(location :sctp_lib_shim.so)",
    ],
    data = [
        ":sctp_lib_shim.so",
        ":sctp_lib_test_binary",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "@bazel_tools//tools/bash/runfiles",
    ],
)

cc_test(
    name = "multinode_timestamp_filter_test",
    srcs = [
        "multinode_timestamp_filter_test.cc",
    ],
    deps = [
        ":multinode_timestamp_filter",
        ":testing_time_converter",
        ":timestamp_filter",
        "//aos/testing:googletest",
    ],
)

cc_binary(
    name = "ping",
    srcs = [
        "ping.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":sctp_server",
        "//aos:init",
        "//aos/events:shm_event_loop",
    ],
)

cc_binary(
    name = "pong",
    srcs = [
        "pong.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":sctp_client",
        "//aos:init",
        "//aos/events:shm_event_loop",
    ],
)

cc_test(
    name = "timestamp_channel_test",
    srcs = ["timestamp_channel_test.cc"],
    data = [":timestamp_channel_test_config"],
    deps = [
        ":timestamp_channel",
        "//aos:configuration",
        "//aos/events:simulated_event_loop",
        "//aos/testing:googletest",
        "//aos/testing:path",
        "//aos/testing:tmpdir",
        "//aos/testing/ping_pong:ping_fbs",
    ],
)

cc_binary(
    name = "sctp_perf",
    srcs = [
        "sctp_perf.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":sctp_client",
        ":sctp_server",
        "//aos:init",
        "//aos/events:shm_event_loop",
        "@com_google_absl//absl/flags:flag",
        "@com_google_absl//absl/flags:usage",
    ],
)

cc_library(
    name = "udp",
    srcs = ["udp.cc"],
    hdrs = ["udp.h"],
    target_compatible_with = select({
        "@platforms//os:linux": [],
        "@platforms//os:osx": [],
        "//conditions:default": ["@platforms//:incompatible"],
    }),
    deps = [
        "//aos:macros",
        "//aos/scoped:scoped_fd",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
    ],
)

cc_test(
    name = "udp_test",
    srcs = ["udp_test.cc"],
    target_compatible_with = select({
        "@platforms//os:linux": [],
        "@platforms//os:osx": [],
        "//conditions:default": ["@platforms//:incompatible"],
    }),
    deps = [
        ":udp",
        "//aos/testing:googletest",
    ],
)
