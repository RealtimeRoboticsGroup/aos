load("@aos//aos/flatbuffers:flatbuffers_python.bzl", "flatbuffer_py_library")
load("@rules_cc//cc:cc_binary.bzl", "cc_binary")
load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")
load("//aos:config.bzl", "aos_config")
load("//aos:flatbuffers.bzl", "cc_static_flatbuffer")
load("//aos/flatbuffers:generate.bzl", "static_flatbuffer")
load("//tools/build_rules:autocxx.bzl", "autocxx_library")
load("//tools/python:defs.bzl", "py_library", "py_test")

package(
    default_applicable_licenses = ["//:license"],
    default_visibility = ["//visibility:public"],
)

static_flatbuffer(
    name = "test_message_fbs",
    srcs = ["test_message.fbs"],
)

cc_static_flatbuffer(
    name = "test_message_schema",
    function = "aos::TestMessageSchema",
    target = ":test_message_fbs_fbs_reflection_out",
)

static_flatbuffer(
    name = "event_loop_fbs",
    srcs = ["event_loop.fbs"],
    deps = ["//aos:configuration_fbs"],
)

cc_static_flatbuffer(
    name = "timing_report_schema",
    function = "aos::timing::ReportSchema",
    target = ":event_loop_fbs_reflection_out",
)

cc_library(
    name = "epoll",
    srcs = [
        "epoll.cc",
    ] + select({
        "@platforms//os:linux": ["epoll_linux.cc"],
        "@platforms//os:macos": ["epoll_darwin.cc"],
        "@platforms//os:windows": ["epoll_windows.cc"],
    }),
    hdrs = ["epoll.h"],
    visibility = ["//visibility:public"],
    deps = [
        "//aos/mutex",
        "//aos/time",
        "@com_google_absl//absl/log:absl_check",
        "@com_google_absl//absl/log:absl_log",
    ],
)

cc_test(
    name = "epoll_test",
    srcs = ["epoll_test.cc"],
    deps = [
        ":epoll",
        "//aos/testing:googletest",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
    ],
)

cc_test(
    name = "timerfd_test",
    srcs = ["timerfd_test.cc"],
    deps = [
        ":epoll",
        "//aos/testing:googletest",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
    ],
)

cc_library(
    name = "context",
    hdrs = [
        "context.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//aos:uuid",
        "//aos/time",
    ],
)

cc_library(
    name = "event_loop",
    srcs = [
        "event_loop.cc",
        "event_loop_event.h",
        "event_loop_tmpl.h",
    ],
    hdrs = [
        "channel_preallocated_allocator.h",
        "event_loop.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":context",
        ":event_loop_fbs",
        ":fetch_result",
        ":timing_statistics",
        "//aos:configuration",
        "//aos:configuration_fbs",
        "//aos:flatbuffers",
        "//aos:ftrace",
        "//aos:realtime",
        "//aos:shared_span",
        "//aos:uuid",
        "//aos/flatbuffers:builder",
        "//aos/ipc_lib:data_alignment",
        "//aos/logging",
        "//aos/time",
        "//aos/util:phased_loop",
        "//aos/util:status",
        "@com_github_google_flatbuffers//:flatbuffers",
        "@com_google_absl//absl/container:btree",
        "@com_google_absl//absl/log:absl_check",
        "@com_google_absl//absl/log:absl_log",
        "@tl-expected//:tl-expected",
    ],
)

cc_binary(
    name = "event_loop_c",
    srcs = [
        "event_loop_c.cc",
        "event_loop_c.h",
    ],
    linkshared = True,
    deps = [
        ":context",
        ":event_loop",
        ":shm_event_loop",
        ":simulated_event_loop",
        "//aos:configuration",
        "//aos:configuration_fbs",
        "//aos:init",
        "@com_github_google_flatbuffers//:flatbuffers",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/log:die_if_null",
    ],
)

py_library(
    name = "event_loop_c_py",
    srcs = [
        "event_loop_c.py",
    ],
    data = [
        ":event_loop_c",
    ],
    deps = [
        "@aos_pip_deps//cffi",
        "@rules_python//python/runfiles",
    ],
)

py_library(
    name = "util_py",
    srcs = [
        "util.py",
    ],
    deps = [
        ":event_loop_c_py",
    ],
)

py_library(
    name = "event_loop_runtime_py",
    srcs = [
        "event_loop_runtime.py",
    ],
    deps = [
        ":event_loop_c_py",
    ],
)

py_library(
    name = "event_loop_py",
    srcs = [
        "event_loop.py",
    ],
    deps = [
        ":event_loop_c_py",
        "//aos/flatbuffers:flatpy",
    ],
)

py_library(
    name = "shm_event_loop_py",
    srcs = [
        "shm_event_loop.py",
    ],
    deps = [
        ":event_loop_c_py",
        ":event_loop_py",
        ":event_loop_runtime_py",
    ],
)

py_test(
    name = "shm_event_loop_test_py",
    srcs = ["shm_event_loop_test.py"],
    data = [
        ":event_loop_py_config",
    ],
    main = "shm_event_loop_test.py",
    deps = [
        ":shm_event_loop_py",
        ":util_py",
        "@aos_pip_deps//absl_py",
    ],
)

py_library(
    name = "simulated_event_loop_py",
    srcs = [
        "simulated_event_loop.py",
    ],
    deps = [
        ":event_loop_c_py",
        ":event_loop_py",
        ":event_loop_runtime_py",
    ],
)

aos_config(
    name = "event_loop_py_config",
    src = "event_loop_py.json",
    flatbuffers = [
        ":test_message_fbs",
    ],
    deps = [
        ":aos_config",
    ],
)

flatbuffer_py_library(
    name = "test_message_fbs_py",
    src = "test_message.fbs",
)

py_test(
    name = "event_loop_test_py",
    srcs = ["event_loop_test.py"],
    data = [
        ":event_loop_py_config",
    ],
    main = "event_loop_test.py",
    deps = [
        ":event_loop_py",
        ":simulated_event_loop_py",
        ":test_message_fbs_py",
        ":util_py",
        "//aos/flatbuffers:flatpy",
        "@aos_pip_deps//absl_py",
    ],
)

cc_library(
    name = "event_loop_runtime_cc",
    srcs = [
        "event_loop_runtime.cc",
    ],
    hdrs = [
        "event_loop_runtime.h",
    ],
    deps = [
        ":event_loop",
        "//aos:for_rust",
        "@com_google_absl//absl/log:absl_check",
        "@com_google_absl//absl/log:absl_log",
        "@crate_index//:cxx_cc",
    ],
)

autocxx_library(
    name = "event_loop_runtime",
    srcs = ["event_loop_runtime.rs"],
    crate_name = "aos_events_event_loop_runtime",
    doctest_deps = [
        "//aos/testing/ping_pong:pong_rust_fbs",
    ],
    gen_tests = False,
    libs = [
        ":event_loop_runtime_cc",
    ],
    override_cc_toolchain = "@llvm_toolchain//:cc-clang-x86_64-linux",
    rs_deps = [
        "//aos/flatbuffers/rust:flatbuffers",
        "@crate_index//:uuid",
        "@crate_index//:futures",
        "@crate_index//:thiserror",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//aos:configuration_rs",
        "//aos:uuid_rs",
    ],
)

autocxx_library(
    name = "event_loop_runtime_test_lib_rs",
    testonly = True,
    srcs = ["event_loop_runtime_test_lib.rs"],
    gen_docs = False,
    libs = [
        ":event_loop",
    ],
    rs_deps = [
        ":event_loop_runtime",
        "//aos/testing/ping_pong:ping_rust_fbs",
        "//aos/testing/ping_pong:pong_rust_fbs",
    ],
)

cc_test(
    name = "event_loop_runtime_test",
    srcs = ["event_loop_runtime_test.cc"],
    data = ["//aos/testing/ping_pong:pingpong_config"],
    deps = [
        ":event_loop_runtime_test_lib_rs",
        ":simulated_event_loop",
        "//aos/testing:googletest",
        "//aos/testing:path",
        "//aos/testing/ping_pong:ping_fbs",
        "//aos/testing/ping_pong:pong_fbs",
        "@com_google_absl//absl/log:absl_check",
        "@com_google_absl//absl/log:absl_log",
    ],
)

aos_config(
    name = "aos_config",
    src = "aos.json",
    flatbuffers = [
        ":event_loop_fbs",
        "//aos/logging:dynamic_log_command_fbs",
        "//aos/logging:log_message_fbs",
        "//aos/starter:starter_fbs",
        "//aos/starter:starter_rpc_fbs",
    ],
)

cc_library(
    name = "timing_statistics",
    srcs = ["timing_statistics.cc"],
    hdrs = ["timing_statistics.h"],
    deps = [
        ":event_loop_fbs",
        "//aos:configuration",
        "//aos/util:error_counter",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
    ],
)

cc_test(
    name = "timing_statistics_test",
    srcs = ["timing_statistics_test.cc"],
    deps = [
        ":timing_statistics",
        "//aos:configuration",
        "//aos:flatbuffers",
        "//aos/testing:googletest",
    ],
)

cc_library(
    name = "shm_event_loop",
    srcs = ["shm_event_loop.cc"],
    hdrs = ["shm_event_loop.h"],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":aos_logging",
        ":epoll",
        ":event_loop",
        ":event_loop_fbs",
        ":test_message_fbs",
        ":timing_statistics",
        "//aos:init",
        "//aos:realtime",
        "//aos/ipc_lib:lockless_queue",
        "//aos/ipc_lib:memory_mapped_queue",
        "//aos/ipc_lib:shm_base",
        "//aos/ipc_lib:signalfd",
        "//aos/stl_mutex",
        "//aos/util:application_name",
        "//aos/util:phased_loop",
        "@com_google_absl//absl/base",
        "@com_google_absl//absl/log:absl_check",
        "@com_google_absl//absl/log:absl_log",
        "@com_google_absl//absl/log:die_if_null",
    ],
)

cc_test(
    name = "shm_event_loop_test",
    srcs = ["shm_event_loop_test.cc"],
    flaky = True,
    shard_count = 50,
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":event_loop_param_test",
        ":shm_event_loop",
        ":test_message_fbs",
    ],
)

cc_library(
    name = "event_loop_param_test",
    testonly = True,
    srcs = ["event_loop_param_test.cc"],
    hdrs = ["event_loop_param_test.h"],
    deps = [
        ":event_loop",
        ":test_message_fbs",
        ":test_message_schema",
        ":timing_report_schema",
        "//aos:realtime",
        "//aos/logging:log_message_fbs",
        "//aos/logging:log_message_schema",
        "//aos/network:message_bridge_client_schema",
        "//aos/network:message_bridge_server_schema",
        "//aos/network:timestamp_schema",
        "//aos/testing:googletest",
    ],
)

cc_test(
    name = "simulated_event_loop_test",
    srcs = ["simulated_event_loop_test.cc"],
    data = [
        "//aos/testing/ping_pong:multinode_pingpong_test_combined_config",
        "//aos/testing/ping_pong:multinode_pingpong_test_split_config",
    ],
    shard_count = 16,
    deps = [
        ":event_loop_param_test",
        ":function_scheduler",
        ":message_counter",
        ":simulated_event_loop",
        "//aos/network:remote_message_fbs",
        "//aos/network:testing_time_converter",
        "//aos/testing:googletest",
        "//aos/testing:path",
        "//aos/testing:tmpdir",
        "//aos/testing/ping_pong:ping_lib",
        "//aos/testing/ping_pong:pong_lib",
        "//aos/util:file",
    ],
)

aos_config(
    name = "event_loop_thread_tester_config",
    src = "event_loop_thread_tester_config_source.json",
    visibility = ["//visibility:public"],
)

cc_binary(
    name = "event_loop_thread_tester",
    testonly = True,
    srcs = [
        "event_loop_thread_tester.cc",
    ],
    data = [
        ":event_loop_thread_tester_config.json",
    ],
    deps = [
        ":shm_event_loop",
        ":simulated_event_loop",
        "//aos:init",
        "//aos:sanitizers",
        "//aos/testing:path",
        "@com_google_absl//absl/flags:flag",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
    ],
)

py_test(
    name = "shm_event_loop_thread_test",
    srcs = [
        "event_loop_thread_test.py",
    ],
    data = [
        ":event_loop_thread_tester",
    ],
    env = {
        "EVENT_LOOP_TYPE": "shm",
    },
    main = "event_loop_thread_test.py",
    deps = [
        "@rules_python//python/runfiles",
    ],
)

py_test(
    name = "simulated_event_loop_thread_test",
    srcs = [
        "event_loop_thread_test.py",
    ],
    data = [
        ":event_loop_thread_tester",
    ],
    env = {
        "EVENT_LOOP_TYPE": "simulated",
    },
    main = "event_loop_thread_test.py",
    deps = [
        "@rules_python//python/runfiles",
    ],
)

cc_library(
    name = "simple_channel",
    srcs = ["simple_channel.cc"],
    hdrs = ["simple_channel.h"],
    deps = [
        "//aos:configuration_fbs",
        "@com_github_google_flatbuffers//:flatbuffers",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/strings",
    ],
)

cc_library(
    name = "simulated_event_loop",
    srcs = [
        "event_scheduler.cc",
        "simulated_event_loop.cc",
        "simulated_network_bridge.cc",
    ],
    hdrs = [
        "event_scheduler.h",
        "simulated_event_loop.h",
        "simulated_network_bridge.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":aos_logging",
        ":epoll",
        ":event_loop",
        ":simple_channel",
        "//aos:init",
        "//aos:realtime",
        "//aos:sanitizers",
        "//aos/events/logging:boot_timestamp",
        "//aos/events/logging:logger_fbs",
        "//aos/ipc_lib:index",
        "//aos/network:message_bridge_client_status",
        "//aos/network:message_bridge_server_status",
        "//aos/network:remote_message_fbs",
        "//aos/network:timestamp_channel",
        "//aos/util:phased_loop",
        "//aos/util:status",
        "@com_google_absl//absl/container:btree",
        "@com_google_absl//absl/flags:flag",
        "@com_google_absl//absl/log:die_if_null",
        "@com_google_absl//absl/log:log_sink",
    ],
)

cc_library(
    name = "simulated_event_loop_for_rust",
    hdrs = ["simulated_event_loop_for_rust.h"],
    deps = [
        ":simulated_event_loop",
        "//aos:for_rust",
        "@crate_index//:cxx_cc",
    ],
)

cc_library(
    name = "shm_event_loop_for_rust",
    hdrs = ["shm_event_loop_for_rust.h"],
    deps = [
        ":event_loop",
        ":simulated_event_loop",
    ],
)

autocxx_library(
    name = "simulated_event_loop_rs",
    srcs = ["simulated_event_loop.rs"],
    crate_name = "aos_events_simulated_event_loop",
    libs = [
        ":simulated_event_loop",
        ":simulated_event_loop_for_rust",
    ],
    override_cc_toolchain = "@llvm_toolchain//:cc-clang-x86_64-linux",
    rs_deps = [
        "//aos/flatbuffers/rust:flatbuffers",
        "//aos:configuration_rust_fbs",
        "//aos:flatbuffers_rs",
        "@crate_index//:futures",
    ],
    test_data = [
        "//aos/testing/ping_pong:multinode_pingpong_test_combined_config",
    ],
    test_deps = [
        "//aos:test_init_rs",
        "//aos/testing:path_rs",
        "//aos/testing/ping_pong:ping_rust_fbs",
        "@crate_index//:futures",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":event_loop_runtime",
        "//aos:configuration_rs",
        "//aos:uuid_rs",
    ],
)

autocxx_library(
    name = "shm_event_loop_rs",
    srcs = ["shm_event_loop.rs"],
    crate_name = "aos_events_shm_event_loop",
    doctest_deps = [
        "//aos/testing/ping_pong:ping_rust_fbs",
        "//aos/testing/ping_pong:pong_rust_fbs",
    ],
    libs = [
        ":shm_event_loop",
        ":shm_event_loop_for_rust",
    ],
    rs_deps = [
        "//aos/flatbuffers/rust:flatbuffers",
        "@crate_index//:futures",
        "//aos:configuration_rust_fbs",
        "//aos:flatbuffers_rs",
    ],
    test_data = ["//aos/testing/ping_pong:pingpong_config"],
    test_deps = [
        "//aos:test_init_rs",
        "//aos/testing:path_rs",
        "//aos/testing/ping_pong:ping_rust_fbs",
        "@crate_index//:futures",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":event_loop_runtime",
        "//aos:configuration_rs",
    ],
)

cc_test(
    name = "event_scheduler_test",
    srcs = ["event_scheduler_test.cc"],
    deps = [
        ":simulated_event_loop",
        "//aos/network:testing_time_converter",
        "//aos/testing:googletest",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
    ],
)

cc_library(
    name = "aos_logging",
    srcs = [
        "aos_logging.cc",
    ],
    hdrs = [
        "aos_logging.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":event_loop",
        "//aos/logging",
        "//aos/logging:log_message_fbs",
    ],
)

cc_library(
    name = "message_counter",
    hdrs = ["message_counter.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":event_loop",
    ],
)

cc_library(
    name = "glib_main_loop",
    srcs = [
        "glib_main_loop.cc",
    ],
    hdrs = [
        "glib_main_loop.h",
    ],
    implementation_deps = [
        "//aos/events:shm_event_loop",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":epoll",
        ":event_loop",
        "//aos/stl_mutex",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
        "@glib//glib",
        "@glib//gobject",
    ],
)

cc_test(
    name = "glib_main_loop_test",
    srcs = [
        "glib_main_loop_test.cc",
    ],
    data = [
        ":aos_config",
    ],
    deps = [
        ":glib_main_loop",
        "//aos:configuration",
        "//aos/events:shm_event_loop",
        "//aos/testing:googletest",
        "//aos/testing:path",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
        "@glib//glib",
        "@glib//gobject",
    ],
)

cc_library(
    name = "timing_report_dump_lib",
    srcs = ["timing_report_dump_lib.cc"],
    hdrs = ["timing_report_dump_lib.h"],
    deps = ["//aos/events:event_loop"],
)

cc_binary(
    name = "timing_report_dump",
    srcs = ["timing_report_dump.cc"],
    deps = [
        ":timing_report_dump_lib",
        "//aos:init",
        "//aos/events/logging:log_reader",
    ],
)

cc_library(
    name = "function_scheduler",
    srcs = [
        "function_scheduler.cc",
    ],
    hdrs = [
        "function_scheduler.h",
    ],
    deps = [":event_loop"],
)

cc_binary(
    name = "aos_timing_report_streamer",
    srcs = ["aos_timing_report_streamer.cc"],
    deps = [
        ":timing_report_dump_lib",
        "//aos:init",
        "//aos/events:shm_event_loop",
    ],
)

cc_library(
    name = "fetch_result",
    hdrs = ["fetch_result.h"],
)
