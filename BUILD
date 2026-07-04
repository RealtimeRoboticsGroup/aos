load("@aspect_rules_js//npm:defs.bzl", "npm_link_package")
load("@aspect_rules_ts//ts:defs.bzl", "ts_config")
load("@bazel_gazelle//:def.bzl", "gazelle", "gazelle_test")
load("@npm//:defs.bzl", "npm_link_all_packages")
load("@rules_license//rules:license.bzl", "license")

# Link npm packages
npm_link_all_packages(name = "node_modules")

exports_files([
    "tsconfig.json",
    "tsconfig.node.json",
    "rollup.config.js",
    # Expose .clang-format so that the static flatbuffer codegen can format its files nicely.
    ".clang-format",
])

license(
    name = "license",
    package_name = "AOS",
    license_kinds = ["@rules_license//licenses/spdx:Apache-2.0"],
    license_text = "LICENSE.txt",
    package_version = "8ca89f37c1327cd59b5f1eb6be3fb7556bc0554f",
)

# The root repo tsconfig
ts_config(
    name = "tsconfig",
    src = "tsconfig.json",
    visibility = ["//visibility:public"],
)

ts_config(
    name = "tsconfig.node",
    src = "tsconfig.node.json",
    visibility = ["//visibility:public"],
    deps = [":tsconfig"],
)

npm_link_package(
    name = "node_modules/flatbuffers",
    src = "@com_github_google_flatbuffers//ts:flatbuffers",
)

npm_link_package(
    name = "node_modules/flatbuffers_reflection",
    src = "@aos//aos/flatbuffers/reflection:flatbuffers_reflection",
)

# gazelle:prefix github.com/RealtimeRoboticsGroup/aos
# gazelle:build_file_name BUILD
# gazelle:proto disable
# gazelle:go_generate_proto false
# gazelle:exclude third_party
# gazelle:exclude external
# gazelle:resolve go github.com/google/flatbuffers/go @com_github_google_flatbuffers//go:go_default_library

gazelle(
    name = "gazelle",
    target_compatible_with = ["//tools/platforms/go:has_support"],
    visibility = ["//tools/lint:__subpackages__"],
)

gazelle_test(
    name = "gazelle_test",
    target_compatible_with = ["//tools/platforms/go:has_support"],
    workspace = "//:BUILD",
)
