load("@rules_pkg//:pkg.bzl", "pkg_tar")
load("//tools/python:defs.bzl", "py_binary")

exports_files(
    ["benchmarking.md"],
    visibility = ["//documentation/aos:__pkg__"],
)

pkg_tar(
    name = "ping_pong_demo",
    srcs = [
        "//aos:aos_dump",
        "//aos/events:aos_timing_report_streamer",
        "//aos/events/logging:logger_main",
        "//aos/starter:aos_starter",
        "//aos/starter:starterd",
        "//documentation/examples/ping_pong:aos_config",
        "//documentation/examples/ping_pong:ping",
        "//documentation/examples/ping_pong:pingpong_config",
        "//documentation/examples/ping_pong:pong",
    ],
)

pkg_tar(
    name = "ping_pong_pi",
    srcs = [
        "//documentation/examples/ping_pong:ping",
        "//documentation/examples/ping_pong:pingpong_config",
        "//documentation/examples/ping_pong:pong",
    ],
)

py_binary(
    name = "starter_demo",
    srcs = ["starter_demo.py"],
    args = [
        "$(rootpath //aos/starter:starterd)",
        "\"$(rootpaths //documentation/examples/ping_pong:pingpong_config)\"",
        "$(rootpath //documentation/examples/ping_pong:ping)",
        "$(rootpath //documentation/examples/ping_pong:pong)",
        "$(rootpath //aos/starter:aos_starter)",
        "$(rootpath //aos:aos_dump)",
        "$(rootpath //aos/events/logging:logger_main)",
        "$(rootpath //aos/events:aos_timing_report_streamer)",
    ],
    data = [
        "//aos:aos_dump",
        "//aos/events:aos_timing_report_streamer",
        "//aos/events/logging:logger_main",
        "//aos/starter:aos_starter",
        "//aos/starter:starterd",
        "//documentation/examples/ping_pong:ping",
        "//documentation/examples/ping_pong:pingpong_config",
        "//documentation/examples/ping_pong:pong",
    ],
)
