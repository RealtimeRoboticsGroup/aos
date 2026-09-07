# Description:
#   Go flatbuffer rules.
#
# Split out for the same reason as rust.bzl: a load() is unconditional, so
# putting this next to flatbuffer_cc_library would force every C++-only
# consumer to have rules_go visible just to parse the file. That also keeps
# the door open to making rules_go a dev_dependency later without touching
# any C++ consumer.

"""Go flatbuffer rules."""

load("@aos//tools/build_rules:clean_dep.bzl", "clean_dep")
load("@io_bazel_rules_go//go:def.bzl", "go_library")
load(
    ":defs.bzl",
    "DEFAULT_FLATC_GO_ARGS",
    "DEFAULT_INCLUDE_PATHS",
    "flatbuffer_library_public",
)

def flatbuffer_go_library(
        name,
        srcs,
        importpath,
        compatible_with = None,
        target_compatible_with = None,
        includes = [],
        include_paths = DEFAULT_INCLUDE_PATHS,
        flatc_args = DEFAULT_FLATC_GO_ARGS,
        visibility = None,
        srcs_filegroup_visibility = None):
    srcs_lib = "%s_srcs" % (name)
    flatc_args = flatc_args + ["--go-namespace", importpath.split("/")[-1]]

    flatbuffer_library_public(
        name = srcs_lib,
        srcs = srcs,
        output_suffix = "_generated.go",
        language_flag = "--go",
        deps = includes,
        include_paths = include_paths,
        flatc_args = flatc_args,
        compatible_with = compatible_with,
        target_compatible_with = target_compatible_with,
        visibility = srcs_filegroup_visibility if srcs_filegroup_visibility != None else ["//visibility:private"],
    )
    go_library(
        name = name,
        srcs = [srcs_lib],
        deps = [clean_dep("@aos_flatbuffers//go")],
        importpath = importpath,
        visibility = visibility,
        compatible_with = compatible_with,
        target_compatible_with = target_compatible_with,
    )
