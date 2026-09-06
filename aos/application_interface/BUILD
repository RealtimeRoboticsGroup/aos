load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "application",
    hdrs = ["application.h"],
)

cc_library(
    name = "application_registry",
    srcs = ["application_registry.cc"],
    hdrs = ["application_registry.h"],
    deps = [
        ":application",
        "//aos/events:event_loop",
        "//aos/util:status",
        "@com_google_absl//absl/functional:any_invocable",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/strings",
    ],
)

cc_library(
    name = "application_macro",
    hdrs = ["application_macro.h"],
    deps = [
        ":application_registry",
        "//aos/events:event_loop",
    ],
)

cc_test(
    name = "application_registry_test",
    srcs = ["application_registry_test.cc"],
    deps = [
        ":application",
        ":application_macro",
        ":application_registry",
        "//aos:json_to_flatbuffer",
        "//aos/events:simulated_event_loop",
        "//aos/testing:googletest",
        "//aos/util:status",
        "@com_google_absl//absl/memory",
    ],
)

cc_library(
    name = "simulated_event_loop_wrapper",
    srcs = ["simulated_event_loop_wrapper.cc"],
    hdrs = ["simulated_event_loop_wrapper.h"],
    deps = [
        ":application",
        ":application_registry",
        "//aos:configuration",
        "//aos/events:simulated_event_loop",
        "//aos/util:status",
    ],
)

cc_test(
    name = "simulated_event_loop_wrapper_test",
    srcs = ["simulated_event_loop_wrapper_test.cc"],
    data = [
        "//aos/application_interface/testdata:aos_config",
        "//aos/application_interface/testdata:aos_multinode_config",
    ],
    deps = [
        ":application_macro",
        ":application_registry",
        ":simulated_event_loop_wrapper",
        "//aos:json_to_flatbuffer",
        "//aos/application_interface/testdata:message_fbs",
        "//aos/events:simulated_event_loop",
        "//aos/testing:googletest",
        "//aos/testing:path",
        "@com_google_absl//absl/memory",
    ],
)
