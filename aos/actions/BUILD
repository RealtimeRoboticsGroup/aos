load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")
load("//aos:config.bzl", "aos_config")
load("//aos/flatbuffers:generate.bzl", "static_flatbuffer")

package(
    default_applicable_licenses = ["//:license"],
    default_visibility = ["//visibility:public"],
)

cc_library(
    name = "action_lib",
    srcs = [
        "actions.cc",
        "actor.cc",
    ],
    hdrs = [
        "actions.h",
        "actor.h",
    ],
    deps = [
        ":actions_fbs",
        "//aos:realtime",
        "//aos/events:event_loop",
        "//aos/logging",
        "//aos/time",
        "//aos/util:phased_loop",
    ],
)

static_flatbuffer(
    name = "actions_fbs",
    srcs = ["actions.fbs"],
)

static_flatbuffer(
    name = "test_action_fbs",
    srcs = ["test_action.fbs"],
)

static_flatbuffer(
    name = "test_action2_fbs",
    srcs = ["test_action2.fbs"],
)

aos_config(
    name = "action_test_config",
    src = "action_test_config_source.json",
    flatbuffers = [
        ":actions_fbs",
        ":test_action_fbs",
        ":test_action2_fbs",
    ],
    deps = [
        "//aos/events:aos_config",
    ],
)

cc_test(
    name = "action_test",
    srcs = [
        "action_test.cc",
    ],
    data = [":action_test_config"],
    deps = [
        ":action_lib",
        ":actions_fbs",
        ":test_action2_fbs",
        ":test_action_fbs",
        "//aos/events:simulated_event_loop",
        "//aos/logging",
        "//aos/testing:googletest",
        "//aos/testing:path",
        "//aos/time",
    ],
)
