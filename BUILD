load("@aspect_rules_js//npm:defs.bzl", "npm_link_package")
load("@aspect_rules_ts//ts:defs.bzl", "ts_config")
load("@npm//:defs.bzl", "npm_link_all_packages")
load("@rules_license//rules:license.bzl", "license")

# Keep this package cheap. //:license is named by default_applicable_licenses
# all over the tree, so every module that depends on AOS loads this BUILD file
# and pays for every load() in it. Gazelle moved to //tools/gazelle for
# exactly that reason; put new tooling in a subpackage rather than here.
#
# npm_link_all_packages has to stay: it links the pnpm workspace rooted at
# //:pnpm-lock.yaml, so it belongs to this package. ts_config has to stay too:
# it copies tsconfig.json to bin, and copy_to_bin requires the file to live in
# the rule's own package. Moving the whole JS workspace under a subdirectory
# is what it would take to shed these loads.

# Link npm packages
npm_link_all_packages(name = "node_modules")

exports_files([
    # gazelle_test in //tools/gazelle anchors its workspace on this file.
    "BUILD",
    "tsconfig.json",
    "tsconfig.node.json",
    "rollup.config.js",
    # Expose .clang-format so that the static flatbuffer codegen can format its files nicely.
    ".clang-format",
    # Shipped inside the AOS SDK tarball.
    "LICENSE.txt",
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
    src = "@aos_flatbuffers//ts:flatbuffers",
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
# gazelle:resolve go github.com/google/flatbuffers/go @aos_flatbuffers//go:go_default_library
