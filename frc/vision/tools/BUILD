load("@rules_cc//cc:cc_binary.bzl", "cc_binary")
load("@rules_cc//cc:cc_test.bzl", "cc_test")
load("//aos:config.bzl", "aos_config")

cc_binary(
    name = "image_replay",
    srcs = [
        "image_replay.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "//aos:configuration",
        "//aos:init",
        "//aos/events:simulated_event_loop",
        "//aos/events/logging:log_reader",
        "//frc/vision:vision_fbs",
        "//third_party:opencv",
    ],
)

cc_binary(
    name = "foxglove_image_converter",
    srcs = ["foxglove_image_converter.cc"],
    visibility = ["//visibility:public"],
    deps = [
        "//aos:init",
        "//aos/events:shm_event_loop",
        "//frc/vision/util:foxglove_image_converter_lib",
    ],
)

aos_config(
    name = "converter_config",
    testonly = True,
    src = "converter_test_config.json",
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

cc_test(
    name = "foxglove_image_converter_test",
    srcs = ["foxglove_image_converter_test.cc"],
    data = [
        ":converter_config",
        "@april_tag_test_image",
    ],
    deps = [
        "//aos:configuration",
        "//aos/events:simulated_event_loop",
        "//aos/testing:googletest",
        "//aos/testing:path",
        "//aos/testing:tmpdir",
        "//frc/vision/util:foxglove_image_converter_lib",
        "@bazel_tools//tools/cpp/runfiles",
        "@com_google_absl//absl/log:die_if_null",
    ],
)
