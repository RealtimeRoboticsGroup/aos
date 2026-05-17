load("@bazel_skylib//rules:common_settings.bzl", "string_flag")
load("//tools/python:defs.bzl", "py_binary")

string_flag(
    name = "rsync_mode",
    build_setting_default = "sandboxed",
    values = [
        "sandboxed",
        "host",
    ],
)

config_setting(
    name = "sandboxed_rsync",
    flag_values = {
        ":rsync_mode": "sandboxed",
    },
)

config_setting(
    name = "sandboxed_rsync_on_linux",
    constraint_values = [
        "@platforms//os:linux",
    ],
    flag_values = {
        ":rsync_mode": "sandboxed",
    },
)

py_binary(
    name = "downloader",
    srcs = [
        "downloader.py",
    ],
    data = select({
        ":sandboxed_rsync_on_linux": [
            "@amd64_debian_sysroot//:rsync-filegroup",
            "@amd64_debian_sysroot//:ssh-filegroup",
        ],
        "//conditions:default": [],
    }),
    main = "downloader.py",
    visibility = ["//visibility:public"],
)
