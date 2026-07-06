load("@aspect_bazel_lib//lib:repositories.bzl", "aspect_bazel_lib_dependencies", "aspect_bazel_lib_register_toolchains", "register_jq_toolchains")
load("@bazel_features//:deps.bzl", "bazel_features_deps")
load("@bazel_skylib//:workspace.bzl", "bazel_skylib_workspace")
load("@bzlmodrio-ni//:maven_cpp_deps.bzl", "setup_legacy_bzlmodrio_ni_cpp_dependencies")
load("@rules_halide//internal:prebuilt_pkg.bzl", "prebuilt_pkg")
load("@rules_halide//internal:versions.bzl", "DEFAULT_VERSION", "SUPPORTED_VERSIONS")
load("@rules_m4//m4:m4.bzl", "m4_register_toolchains")
load("@rules_pkg//:deps.bzl", "rules_pkg_dependencies")
load("@rules_python//python:repositories.bzl", "py_repositories", "python_register_toolchains")

def dependencies_phase1():
    native.register_toolchains("//tools/python:python_workspace_toolchain")
    version_info = SUPPORTED_VERSIONS[DEFAULT_VERSION]
    prebuilt_pkg(
        name = "halide_prebuilt_pkg",
        build_file = "@rules_halide//internal:BUILD.prebuilt_pkg",
        urls = {platform: [info["url"]] for platform, info in version_info.items()},
        sha256 = {platform: info["sha256"] for platform, info in version_info.items()},
        strip_prefix = {platform: info["strip_prefix"] for platform, info in version_info.items()},
    )
    bazel_skylib_workspace()

    aspect_bazel_lib_dependencies()

    aspect_bazel_lib_register_toolchains()

    register_jq_toolchains()

    py_repositories()

    bazel_features_deps()

    python_register_toolchains(
        name = "python_3_10",
        python_version = "3.10",
    )

    python_register_toolchains(
        name = "python_3_13",
        python_version = "3.13",
    )

    m4_register_toolchains(version = "1.4.18")
    setup_legacy_bzlmodrio_ni_cpp_dependencies()

    rules_pkg_dependencies()
