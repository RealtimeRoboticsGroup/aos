load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")

package(
    default_applicable_licenses = ["//:license"],
    default_visibility = ["//visibility:public"],
)

cc_library(
    name = "ring_buffer",
    hdrs = [
        "ring_buffer.h",
    ],
)

cc_test(
    name = "ring_buffer_test",
    srcs = [
        "ring_buffer_test.cc",
    ],
    deps = [
        ":ring_buffer",
        "//aos/testing:googletest",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
    ],
)

cc_library(
    name = "priority_queue",
    hdrs = [
        "priority_queue.h",
    ],
)

cc_test(
    name = "priority_queue_test",
    srcs = [
        "priority_queue_test.cc",
    ],
    deps = [
        ":priority_queue",
        "//aos/testing:googletest",
    ],
)

cc_library(
    name = "inlined_vector",
    hdrs = [
        "inlined_vector.h",
    ],
    deps = [
        "@com_google_absl//absl/container:inlined_vector",
    ],
)

cc_test(
    name = "inlined_vector_test",
    srcs = [
        "inlined_vector_test.cc",
    ],
    deps = [
        ":inlined_vector",
        "//aos:realtime",
        "//aos:sanitizers",
        "//aos/testing:googletest",
        "@com_google_absl//absl/flags:reflection",
    ],
)

cc_library(
    name = "sized_array",
    hdrs = [
        "sized_array.h",
    ],
    deps = [
        "@com_google_absl//absl/container:inlined_vector",
    ],
)

cc_test(
    name = "sized_array_test",
    srcs = [
        "sized_array_test.cc",
    ],
    deps = [
        ":sized_array",
        "//aos/testing:googletest",
    ],
)

cc_library(
    name = "error_list",
    hdrs = [
        "error_list.h",
    ],
    deps = [
        ":sized_array",
        "//aos:flatbuffers",
    ],
)

cc_test(
    name = "error_list_test",
    srcs = [
        "error_list_test.cc",
    ],
    deps = [
        ":error_list",
        "//aos:json_to_flatbuffer_fbs",
        "//aos/testing:googletest",
    ],
)

cc_library(
    name = "resizeable_buffer",
    hdrs = [
        "resizeable_buffer.h",
    ],
    deps = [
        "@com_google_absl//absl/log:absl_check",
    ],
)

cc_test(
    name = "resizeable_buffer_test",
    srcs = [
        "resizeable_buffer_test.cc",
    ],
    deps = [
        ":resizeable_buffer",
        "//aos/testing:googletest",
        "@com_google_absl//absl/types:span",
    ],
)
