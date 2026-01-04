load("@rules_cc//cc:cc_binary.bzl", "cc_binary")
load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")

cc_library(
    name = "aos_sync",
    srcs = [
        "aos_sync.cc",
    ],
    hdrs = [
        "aos_sync.h",
    ],
    linkopts = [
        "-lpthread",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":shm_observers",
        "//aos:macros",
        "//aos/util:compiler_memory_barrier",
        "@com_google_absl//absl/base",
        "@com_google_absl//absl/log:absl_check",
        "@com_google_absl//absl/log:absl_log",
    ],
)

cc_library(
    name = "core_lib",
    srcs = [
        "core_lib.c",
    ],
    hdrs = [
        "core_lib.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":aos_sync",
        ":shared_mem_types",
    ],
)

cc_library(
    name = "shared_mem",
    srcs = [
        "shared_mem.cc",
    ],
    hdrs = [
        "shared_mem.h",
    ],
    linkopts = [
        "-lrt",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":aos_sync",
        ":core_lib",
        ":shared_mem_types",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
    ],
)

cc_library(
    # TODO(Brian): This should be shared_mem{,.h}, and the other one should be
    # shared_mem_init{,.cc,.h}.
    name = "shared_mem_types",
    hdrs = [
        "shared_mem_types.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":aos_sync",
    ],
)

cc_test(
    name = "ipc_stress_test",
    srcs = [
        "ipc_stress_test.cc",
    ],
    tags = [
        "manual",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":core_lib",
        "//aos:die",
        "//aos/libc:aos_strsignal",
        "//aos/libc:dirname",
        "//aos/logging",
        "//aos/mutex",
        "//aos/testing:googletest",
        "//aos/testing:test_shm",
        "//aos/time",
    ],
)

cc_binary(
    name = "ipc_comparison",
    srcs = [
        "ipc_comparison.cc",
    ],
    linkopts = ["-lrt"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":event",
        "//aos:condition",
        "//aos:init",
        "//aos/logging",
        "//aos/mutex",
        "@com_google_absl//absl/flags:flag",
    ],
)

cc_library(
    name = "signalfd",
    srcs = [
        "signalfd.cc",
    ],
    hdrs = [
        "signalfd.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "//aos:sanitizers",
        "@com_google_absl//absl/log:absl_check",
        "@com_google_absl//absl/log:absl_log",
    ],
)

cc_test(
    name = "signalfd_test",
    srcs = [
        "signalfd_test.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":signalfd",
        "//aos/testing:googletest",
        "//aos/testing:test_logging",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
    ],
)

cc_library(
    name = "index32",
    srcs = ["index.cc"],
    hdrs = ["index.h"],
    defines = [
        "AOS_QUEUE_ATOMIC_SIZE=32",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":shm_observers",
        "@com_google_absl//absl/log:absl_check",
        "@com_google_absl//absl/log:absl_log",
    ],
)

cc_test(
    name = "index32_test",
    srcs = ["index_test.cc"],
    deps = [
        ":index32",
        "//aos/testing:googletest",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
    ],
)

cc_library(
    name = "index",
    srcs = ["index.cc"],
    hdrs = ["index.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":shm_observers",
        "@com_google_absl//absl/log:absl_check",
        "@com_google_absl//absl/log:absl_log",
    ],
)

cc_test(
    name = "index_test",
    srcs = ["index_test.cc"],
    deps = [
        ":index",
        "//aos/testing:googletest",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
    ],
)

# Define two versions of the lockless_queue library so that we can readily write tests
# which exercise both with and without AOS_IPC_LIB_LOCKLESS_QUEUE_HAS_ATOMIC_TIME_POINT.
# Having to define every target twice is a bit awkward, but the main alternative (using
# build settings + configuration transitions), while a bit more theoretically elegant, both
# tends to increase the size of the build graph more significantly and is just harder to
# actually comprehend/deal with.

cc_library(
    name = "lockless_queue",
    visibility = ["//visibility:public"],
    deps = select({
        "//tools/platforms/hardware:roborio": [":lockless_queue_no_atomic_times"],
        "//conditions:default": [":lockless_queue_atomic_times"],
    }),
)

[
    (
        cc_library(
            name = "lockless_queue" + suffix,
            srcs = [
                "lockless_queue.cc",
                "lockless_queue_memory.h",
                "robust_ownership_tracker.cc",
            ],
            hdrs = [
                "lockless_queue.h",
                "robust_ownership_tracker.h",
            ],
            defines = ["AOS_IPC_LIB_LOCKLESS_QUEUE_HAS_ATOMIC_TIME_POINT"] if suffix != "_no_atomic_times" else [],
            target_compatible_with = ["@platforms//os:linux"],
            deps = [
                ":aos_sync",
                ":data_alignment",
                ":index",
                "//aos:realtime",
                "//aos:uuid",
                "//aos/containers:ring_buffer",
                "//aos/events:context",
                "//aos/time",
                "//aos/util:compiler_memory_barrier",
                "//aos/util:proc_stat",
                "@com_google_absl//absl/flags:flag",
                "@com_google_absl//absl/log",
                "@com_google_absl//absl/log:check",
                "@com_google_absl//absl/strings",
                "@com_google_absl//absl/types:span",
            ],
        ),
        cc_library(
            name = "multichannel_queue_racer" + suffix,
            testonly = True,
            srcs = [
                "multichannel_queue_racer.cc",
            ],
            hdrs = [
                "multichannel_queue_racer.h",
            ],
            # For the same reasons that we generally run this on the larger worker pools
            # we also find that we generally cannot as reliably trigger the expected race
            # conditions on our arm workers, so we disable certain aspects of the test in
            # that case.
            defines = select({
                "@platforms//cpu:x86_64": ["AOS_IPC_LIB_TEST_CAN_RELIABLY_TRIGGER_RACES"],
                "//conditions:default": [],
            }),
            deps = [
                ":lockless_queue" + suffix,
                "//aos/ipc_lib:event",
                "//aos/testing:googletest",
                "@com_google_absl//absl/log",
                "@com_google_absl//absl/log:check",
            ],
        ),
        cc_test(
            name = "multichannel_queue_racer_test" + suffix,
            srcs = ["multichannel_queue_racer_test.cc"],
            tags = ["no-remote-exec"],
            deps = [
                ":multichannel_queue_racer" + suffix,
                "//aos/testing:googletest",
            ],
        ),
        cc_library(
            name = "queue_racer" + suffix,
            testonly = True,
            srcs = [
                "queue_racer.cc",
            ],
            hdrs = [
                "queue_racer.h",
            ],
            target_compatible_with = ["@platforms//os:linux"],
            deps = [
                ":event",
                ":lockless_queue" + suffix,
                "//aos/testing:googletest",
            ],
        ),
        cc_library(
            name = "lockless_queue_stepping" + suffix,
            testonly = True,
            srcs = [
                "lockless_queue_stepping.cc",
            ],
            hdrs = ["lockless_queue_stepping.h"],
            deps = [
                ":lockless_queue" + suffix,
                ":shm_observers",
                "//aos/libc:aos_strsignal",
                "//aos/testing:prevent_exit",
                "@com_google_googletest//:gtest",
            ],
        ),
        cc_test(
            name = "robust_ownership_tracker_test" + suffix,
            srcs = ["robust_ownership_tracker_test.cc"],
            target_compatible_with = ["@platforms//os:linux"],
            deps = [
                ":lockless_queue" + suffix,
                "//aos/testing:googletest",
            ],
        ),
        cc_library(
            name = "lockless_queue_test_utils" + suffix,
            testonly = True,
            srcs = ["lockless_queue_test_utils.cc"],
            hdrs = ["lockless_queue_test_utils.h"],
            deps = [
                ":event",
                ":lockless_queue" + suffix,
                ":signalfd",
                "//aos:realtime",
                "//aos/events:epoll",
                "//aos/testing:googletest",
            ],
        ),
        cc_test(
            name = "lockless_queue_test" + suffix,
            srcs = ["lockless_queue_test.cc"],
            # We don't want all the signal handlers registered, otherwise we can't fork.
            # Also, start each of the different suffix at a different core
            args = ["--nobacktrace"] + (["--start_core_index=4"] if suffix != "_no_atomic_times" else ["--start_core_index=6"]),
            # For the same reasons that we generally run this on the larger worker pools
            # we also find that we generally cannot as reliably trigger certain expected race
            # conditions on our arm workers, so we disable certain aspects of the test in
            # that case.
            defines = select({
                "@platforms//cpu:x86_64": ["AOS_IPC_LIB_TEST_CAN_RELIABLY_TRIGGER_RACES"],
                "//conditions:default": [],
            }),
            shard_count = 10,
            target_compatible_with = ["@platforms//os:linux"],
            deps = [
                ":event",
                ":lockless_queue" + suffix,
                ":lockless_queue_stepping" + suffix,
                ":lockless_queue_test_utils" + suffix,
                ":queue_racer" + suffix,
                ":signalfd",
                "//aos/events:epoll",
                "//aos/testing:googletest",
                "//aos/util:phased_loop",
            ],
        ),
        cc_test(
            name = "lockless_queue_wrap_test" + suffix,
            timeout = "eternal",
            srcs = ["lockless_queue_wrap_test.cc"],
            # We don't want all the signal handlers registered, otherwise we can't fork.
            # Also, start each of the different suffix at a different core
            args = ["--nobacktrace"] + (["--start_core_index=0"] if suffix != "_no_atomic_times" else ["--start_core_index=2"]),
            tags = ["very_slow_tests"],
            deps = [
                ":lockless_queue_test_utils" + suffix,
                ":queue_racer" + suffix,
            ],
        ),
        cc_test(
            name = "lockless_queue_death_test" + suffix,
            srcs = ["lockless_queue_death_test.cc"],
            # We don't want all the signal handlers registered, otherwise we can't fork.
            args = ["--nobacktrace"],
            target_compatible_with = ["@platforms//os:linux"],
            deps = [
                ":event",
                ":lockless_queue" + suffix,
                ":lockless_queue_stepping" + suffix,
                ":queue_racer" + suffix,
                ":shm_observers",
                ":signalfd",
                "//aos/events:epoll",
                "//aos/libc:aos_strsignal",
                "//aos/testing:googletest",
                "//aos/testing:prevent_exit",
                "//aos/testing:test_logging",
            ],
        ),
    )
    for suffix in [
        "_no_atomic_times",
        "_atomic_times",
    ]
]

cc_library(
    name = "memory_mapped_queue",
    srcs = ["memory_mapped_queue.cc"],
    hdrs = ["memory_mapped_queue.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":index",
        ":lockless_queue",
        "//aos:configuration",
        "//aos/util:file",
        "@com_github_google_flatbuffers//:flatbuffers",
        "@com_google_absl//absl/log:absl_check",
        "@com_google_absl//absl/log:absl_log",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/types:span",
    ],
)

cc_library(
    name = "data_alignment",
    hdrs = [
        "data_alignment.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "@com_google_absl//absl/log:absl_check",
    ],
)

cc_library(
    name = "latency_lib",
    srcs = ["latency_lib.cc"],
    hdrs = ["latency_lib.h"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "//aos:realtime",
        "//aos/logging",
        "//aos/time",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
    ],
)

cc_binary(
    name = "signal_stress",
    srcs = [
        "signal_stress.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":latency_lib",
        "//aos:init",
        "//aos/events:epoll",
        "@com_google_absl//absl/flags:flag",
    ],
)

cc_binary(
    name = "futex_latency",
    srcs = [
        "futex_latency.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":latency_lib",
        "//aos:condition",
        "//aos:init",
        "//aos/mutex",
        "@com_google_absl//absl/flags:flag",
    ],
)

cc_binary(
    name = "named_pipe_latency",
    srcs = [
        "named_pipe_latency.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":latency_lib",
        "//aos:init",
        "//aos/events:epoll",
        "@com_google_absl//absl/flags:flag",
    ],
)

cc_binary(
    name = "eventfd_latency",
    srcs = [
        "eventfd_latency.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":latency_lib",
        "//aos:init",
        "//aos/events:epoll",
        "@com_google_absl//absl/flags:flag",
    ],
)

cc_binary(
    name = "print_lockless_queue_memory",
    srcs = [
        "print_lockless_queue_memory.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":lockless_queue",
    ],
)

cc_library(
    name = "event",
    srcs = [
        "event.cc",
    ],
    hdrs = [
        "event.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "//aos/ipc_lib:aos_sync",
        "//aos/time",
        "//aos/type_traits",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
    ],
)

cc_test(
    name = "event_test",
    srcs = [
        "event_test.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":event",
        "//aos/testing:googletest",
        "//aos/testing:test_logging",
        "//aos/time",
    ],
)

cc_library(
    name = "shm_observers",
    srcs = [
        "shm_observers.cc",
    ],
    hdrs = [
        "shm_observers.h",
    ],
)

cc_library(
    name = "shm_base",
    srcs = ["shm_base.cc"],
    hdrs = ["shm_base.h"],
    visibility = ["//visibility:public"],
    deps = [
        "@com_google_absl//absl/flags:flag",
    ],
)

cc_library(
    name = "memory_estimation",
    srcs = ["memory_estimation.cc"],
    hdrs = ["memory_estimation.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":lockless_queue",
        ":memory_mapped_queue",
        "//aos:configuration",
    ],
)
