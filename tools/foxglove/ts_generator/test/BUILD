load("@rules_shell//shell:sh_binary.bzl", "sh_binary")
load("@rules_shell//shell:sh_test.bzl", "sh_test")
load("//aos/flatbuffers:generate.bzl", "static_flatbuffer")
load("//tools/foxglove/ts_generator:foxglove_ts.bzl", "foxglove_ts_bundle")

# A flatbuffer that test_fbs imports, used to exercise cross-file Foxglove TS
# generation (the bundle macro should emit a .ts file for it alongside the .ts
# file for test.fbs).
static_flatbuffer(
    name = "imported_fbs",
    srcs = ["imported.fbs"],
)

# A flatbuffer for validating the output of the generator. See the golden file
# comparison below.
static_flatbuffer(
    name = "test_fbs",
    srcs = ["test.fbs"],
    deps = [":imported_fbs"],
)

# A multi-source static_flatbuffer used to exercise the foxglove TS aspect's
# same-node sibling --import wiring: multi_b.fbs references a type declared
# in multi_a.fbs, and the generated multi_b_foxglove.ts must import that
# type from the sibling multi_a_foxglove.ts emitted by the same node.
static_flatbuffer(
    name = "multi_fbs",
    srcs = [
        "multi_a.fbs",
        "multi_b.fbs",
    ],
)

# A flatbuffer that depends on a type from an external workspace
# (@com_github_foxglove_schemas) to exercise cross-workspace Foxglove TS generation.
static_flatbuffer(
    name = "external_dep_fbs",
    srcs = ["external_dep.fbs"],
    deps = ["@com_github_foxglove_schemas//:schemas"],
)

# Generates a .ts file for each .fbs in the transitive dep closure of the
# listed targets (test_foxglove.ts + imported_foxglove.ts from `:test_fbs`,
# multi_a_foxglove.ts + multi_b_foxglove.ts from the multi-source
# `:multi_fbs`, and external_dep_foxglove.ts + the foxglove SDK schemas from
# `:external_dep_fbs`). Their outputs are compared against the checked-in
# golden files by compare_generated_files below.  Using foxglove_ts_bundle
# (instead of invoking the generator directly) keeps this package exercising
# the public macro that downstream callers actually use.
foxglove_ts_bundle(
    name = "test_foxglove_ts",
    out_dir = "generated",
    deps = [
        ":external_dep_fbs",
        ":multi_fbs",
        ":test_fbs",
    ],
)

sh_test(
    name = "compare_generated_files",
    srcs = ["compare_generated_files.sh"],
    args = [
        # Used by the script to locate the golden/ directory in runfiles; the
        # script then walks it to pair each golden with a matching generated
        # file by basename and fails if either set has files the other lacks.
        "$(rootpath golden/tools_foxglove_ts_generator_test__test_foxglove.ts)",
        "$(rootpaths :test_foxglove_ts)",
    ],
    data = [
        "golden/com_github_foxglove_schemas__Pose_foxglove.ts",
        "golden/com_github_foxglove_schemas__Quaternion_foxglove.ts",
        "golden/com_github_foxglove_schemas__Vector3_foxglove.ts",
        "golden/tools_foxglove_ts_generator_test__external_dep_foxglove.ts",
        "golden/tools_foxglove_ts_generator_test__imported_foxglove.ts",
        "golden/tools_foxglove_ts_generator_test__multi_a_foxglove.ts",
        "golden/tools_foxglove_ts_generator_test__multi_b_foxglove.ts",
        "golden/tools_foxglove_ts_generator_test__test_foxglove.ts",
        ":test_foxglove_ts",
    ],
)

sh_binary(
    name = "copy_test_foxglove_golden",
    srcs = ["copy_test_foxglove_golden.sh"],
    args = ["$(locations :test_foxglove_ts)"],
    data = [":test_foxglove_ts"],
)
