load("//tools/platforms:host_platform.bzl", "host_compatible_with")
load("//tools/python:defs.bzl", "py_binary")
load(":packages.bzl", "generate_deb_tarball")
load(
    ":phoenix6.bzl",
    phoenix6_debs = "files",
)

package(default_visibility = ["//visibility:public"])

py_binary(
    name = "download_packages",
    srcs = [
        "download_packages.py",
    ],
    main = "download_packages.py",
    target_compatible_with = host_compatible_with([
        "@platforms//os:linux",
        "@platforms//cpu:x86_64",
    ]),
)

generate_deb_tarball(
    name = "phoenix6",
    files = phoenix6_debs,
)

exports_files([
    "curl.BUILD",
    "BUILD.zlib.bazel",
])
