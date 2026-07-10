load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")
load("@rules_rust_bindgen//:defs.bzl", "rust_bindgen_library")
load("//tools/build_rules:clean_dep.bzl", "aos_repo_name")
load("//tools/rust:defs.bzl", "rust_library")

cc_library(
    name = "googletest",
    testonly = True,
    srcs = [
        "gtest_main.cc",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//aos:init",
        "//aos/testing:tmpdir",
        "@com_google_absl//absl/flags:flag",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
        "@com_google_googletest//:gtest",
    ],
)

cc_library(
    name = "test_logging",
    testonly = True,
    srcs = [
        "test_logging.cc",
    ],
    hdrs = [
        "test_logging.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":googletest",
        "//aos/logging",
        "//aos/stl_mutex",
        "@com_google_absl//absl/base",
    ],
)

cc_test(
    name = "test_logging_test",
    srcs = [
        "test_logging_test.cc",
    ],
    deps = [
        ":googletest",
        ":test_logging",
        "//aos/logging",
    ],
)

cc_library(
    name = "test_child",
    testonly = True,
    srcs = [
        "test_child.cc",
    ],
    hdrs = [
        "test_child.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "@com_google_absl//absl/log:absl_check",
        "@com_google_absl//absl/log:absl_log",
    ] + select({
        "@platforms//os:windows": [],
        "//conditions:default": [":prevent_exit"],
    }),
)

cc_library(
    name = "prevent_exit",
    testonly = True,
    srcs = [
        "prevent_exit.cc",
    ],
    hdrs = [
        "prevent_exit.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
    ],
)

cc_library(
    name = "test_shm",
    testonly = True,
    srcs = [
        "test_shm.cc",
    ],
    hdrs = [
        "test_shm.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":test_logging",
        "@com_google_absl//absl/log:absl_check",
    ],
)

cc_library(
    name = "random_seed",
    testonly = True,
    srcs = [
        "random_seed.cc",
    ],
    hdrs = [
        "random_seed.h",
    ],
    visibility = ["//visibility:public"],
)

# rust_bindgen_library does not forward testonly to the rust_bindgen target it
# creates, so cc_lib cannot be the testonly :tmpdir.  Give bindgen the header on
# its own; :tmpdir still provides the implementation.
cc_library(
    name = "tmpdir_c_hdrs",
    hdrs = ["tmpdir_c.h"],
    visibility = ["//visibility:public"],
)

rust_bindgen_library(
    name = "tmpdir_c_rs",
    bindgen_flags = [
        "--allowlist-file=.*/tmpdir_c.h",
        "--no-layout-tests",
        "--merge-extern-blocks",
    ],
    cc_lib = ":tmpdir_c_hdrs",
    crate_name = "aos_testing_tmpdir_c",
    header = "tmpdir_c.h",
    # See //aos/events:event_loop_c_rs.
    merge_cc_lib_objects_into_rlib = False,
    target_compatible_with = ["//tools/platforms/rust:has_support"],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "tmpdir",
    testonly = True,
    srcs = ["tmpdir.cc"],
    hdrs = [
        "tmpdir.h",
        "tmpdir_c.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//aos:realtime",
        "//aos/ipc_lib:shm_base",
        "@com_google_absl//absl/flags:flag",
        "@com_google_absl//absl/log:check",
    ],
)

cc_library(
    name = "flatbuffer_eq",
    testonly = True,
    hdrs = [
        "flatbuffer_eq.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//aos:flatbuffer_merge",
        "//aos:flatbuffers",
        "//aos:json_to_flatbuffer",
        "@com_google_googletest//:gtest",
    ],
)

cc_library(
    name = "path",
    srcs = ["path.cc"],
    hdrs = ["path.h"],
    local_defines = ["AOS_REPO_NAME=\\\"" + aos_repo_name() + "\\\""],
    visibility = ["//visibility:public"],
    deps = [
        "@com_google_absl//absl/strings",
    ],
)

rust_library(
    name = "path_rs",
    srcs = ["path.rs"],
    crate_name = "aos_testing_path",
    rustc_env = {"AOS_REPO_NAME": aos_repo_name()},
    visibility = ["//visibility:public"],
)

rust_library(
    name = "aos_rs",
    testonly = True,
    srcs = ["aos.rs"],
    crate_name = "aos",
    gen_docs = False,
    gen_doctests = False,
    visibility = ["//visibility:public"],
    deps = [
        ":path_rs",
        "//aos:aos_rs",
        "//aos:test_init_rs",
    ],
)
