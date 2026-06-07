load("@aos//aos/flatbuffers:typescript.bzl", "flatbuffer_ts_library")
load("@com_google_protobuf//bazel:cc_proto_library.bzl", "cc_proto_library")
load("@com_google_protobuf//bazel:proto_library.bzl", "proto_library")
load("@rules_cc//cc:cc_binary.bzl", "cc_binary")
load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")
load("@rules_rust//rust:defs.bzl", "rust_binary", "rust_library")
load("//tools/build_rules:apache.bzl", "apache_wrapper")
load("//tools/build_rules:autocxx.bzl", "autocxx_library")
load("//tools/build_rules:js.bzl", "ts_project")
load("//tools/python:defs.bzl", "py_binary", "py_test")

cc_test(
    name = "gflags_build_test",
    size = "small",
    srcs = [
        "gflags.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "@com_google_absl//absl/flags:flag",
    ],
)

# Smoke tests for the toolchain sanitizers. They pass trivially in a normal
# build; their value is exercising the instrumented libc++ under
# `--config=msan` (and the other `--config=asan|ubsan|tsan` builds).
cc_test(
    name = "msan_smoke_build_test",
    size = "small",
    srcs = [
        "msan_smoke.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
)

cc_test(
    name = "msan_runtime_smoke_build_test",
    size = "small",
    srcs = [
        "msan_runtime_smoke.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
)

cc_binary(
    name = "tcmalloc_build_test_binary",
    srcs = [
        "tcmalloc.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
)

sh_test(
    name = "tcmalloc_build_test",
    size = "small",
    srcs = [
        "tcmalloc_test.sh",
    ],
    data = [
        ":tcmalloc_build_test_binary",
    ],
    # TODO(austin): Turn this back on when we get memory instrumentation figured out for tcmalloc for --die_on_malloc
    tags = ["manual"],
    target_compatible_with = ["@platforms//os:linux"],
)

proto_library(
    name = "proto_build_test_library_proto",
    srcs = ["proto.proto"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":proto_build_test_library_base",
        "@com_google_protobuf//:empty_proto",
    ],
)

cc_proto_library(
    name = "proto_build_test_library",
    deps = [":proto_build_test_library_proto"],
)

proto_library(
    name = "proto_build_test_library_base",
    srcs = ["proto_base.proto"],
    target_compatible_with = ["@platforms//os:linux"],
)

cc_test(
    name = "proto_build_test",
    size = "small",
    srcs = [
        "proto.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":proto_build_test_library",
        "//aos/testing:googletest",
    ],
)

flatbuffer_ts_library(
    name = "test_ts_fbs",
    srcs = ["test.fbs"],
    namespace = "aos.examples",
    tables = [
        "Foo",
        "Bar",
    ],
    target_compatible_with = ["@platforms//cpu:x86_64"],
    visibility = [":__subpackages__"],
)

py_test(
    name = "python3_opencv",
    srcs = ["python_opencv.py"],
    main = "python_opencv.py",
    tags = [
        # This needs a GPU, which means we need the GPU to be accessible inside
        # the container.  Rather than solve that right now, just run it under
        # linux-sandbox on the host which has a GPU already.
        "no-remote-exec",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = ["@pip_deps//opencv_python"],
)

py_test(
    name = "python_jinja2",
    srcs = ["python_jinja2.py"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = ["@pip_deps//jinja2"],
)

py_binary(
    name = "dummy_http_server",
    srcs = ["dummy_http_server.py"],
    target_compatible_with = ["@platforms//cpu:x86_64"],
)

apache_wrapper(
    name = "apache_https_demo",
    binary = ":dummy_http_server",
    target_compatible_with = ["@platforms//cpu:x86_64"],
)

rust_library(
    name = "hello_lib",
    srcs = ["hello_lib.rs"],
    target_compatible_with = ["//tools/platforms/rust:has_support"],
)

rust_binary(
    name = "rust_hello",
    srcs = ["rust_hello.rs"],
    target_compatible_with = ["//tools/platforms/rust:has_support"],
    deps = [":hello_lib"],
)

ts_project(
    name = "build_tests_ts",
    srcs = ["basic.ts"],
)

rust_library(
    name = "rust_in_cc_rs",
    srcs = ["rust_in_cc.rs"],
    target_compatible_with = ["//tools/platforms/rust:has_support"],
)

cc_test(
    name = "rust_in_cc",
    srcs = ["rust_in_cc.cc"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [":rust_in_cc_rs"],
)

cc_library(
    name = "hello_autocxx_cc",
    hdrs = [
        "hello_autocxx.h",
    ],
)

autocxx_library(
    name = "hello_autocxx",
    srcs = ["hello_autocxx.rs"],
    libs = [":hello_autocxx_cc"],
    override_cc_toolchain = "@llvm_toolchain//:cc-clang-x86_64-linux",
    target_compatible_with = ["//tools/platforms/rust:has_support"],
)

py_test(
    name = "upstream_python_test",
    srcs = [
        "upstream_python_test.py",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "@pip_deps//matplotlib",
        "@pip_deps//numpy",
        "@pip_deps//scipy",
    ],
)

py_binary(
    name = "pygobject_example",
    srcs = ["pygobject_example.py"],
    deps = [
        "@pip_deps//pygobject",
    ],
)

py_binary(
    name = "matplotlib_example",
    srcs = ["matplotlib_example.py"],
    deps = [
        "@pip_deps//matplotlib",
        "@pip_deps//pygobject",
    ],
)
