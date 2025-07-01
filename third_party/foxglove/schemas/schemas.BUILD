load("@com_github_google_flatbuffers//:build_defs.bzl", "DEFAULT_FLATC_ARGS", "flatbuffer_cc_library")
load("@aos//aos/flatbuffers:generate.bzl", "static_flatbuffer")
load("@rules_license//rules:license.bzl", "license")
load("@aspect_bazel_lib//lib:copy_file.bzl", "copy_file")

package(default_applicable_licenses = [":license"])

license(
    name = "license",
    package_name = "Foxglove Schemas",
    license_kinds = ["@rules_license//licenses/spdx:MIT"],
    license_text = "LICENSE.md",
    package_version = "7658786f354af2fab053b7151b364dd751c0c0f",
)

FLATC_ARGS = [arg for arg in DEFAULT_FLATC_ARGS if arg != "--require-explicit-ids"]

SCHEMAS_FOLDER = "schemas/flatbuffer/"

SCHEMAS = [f.split("/")[-1] for f in glob([SCHEMAS_FOLDER + "*.fbs"])]

[copy_file(
    name = "copy_" + f,
    src = SCHEMAS_FOLDER + f,
    out = f,
) for f in SCHEMAS]

NON_TABLE_SCHEMAS = [
    "Duration.fbs",
    "Time.fbs",
]

TABLE_SCHEMAS = [f for f in SCHEMAS if f not in NON_TABLE_SCHEMAS]

static_flatbuffer(
    name = "non_table_schemas",
    srcs = NON_TABLE_SCHEMAS,
    flatc_args = FLATC_ARGS,
    visibility = ["//visibility:public"],
)

static_flatbuffer(
    name = "schemas",
    srcs = TABLE_SCHEMAS,
    flatc_args = FLATC_ARGS,
    visibility = ["//visibility:public"],
    deps = [":non_table_schemas"],
)

load("@aos//aos:flatbuffers.bzl", "cc_static_flatbuffer")

[cc_static_flatbuffer(
    name = filename[:-4] + "_schema",
    bfbs_name = filename[:-4] + ".bfbs",
    function = "foxglove::" + filename[:-4] + "Schema",
    target = ":schemas_reflection_out",
    visibility = ["//visibility:public"],
) for filename in TABLE_SCHEMAS]

[cc_static_flatbuffer(
    name = filename[:-4] + "_schema",
    bfbs_name = filename[:-4] + ".bfbs",
    function = "foxglove::" + filename[:-4] + "Schema",
    target = ":non_table_schemas_reflection_out",
    visibility = ["//visibility:public"],
) for filename in NON_TABLE_SCHEMAS]
