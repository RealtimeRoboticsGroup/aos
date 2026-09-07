load("@bazel_gazelle//:def.bzl", "gazelle", "gazelle_test")

package(default_applicable_licenses = ["//:license"])

# Gazelle lives here rather than in the root package. The root BUILD is loaded
# by every module that depends on AOS -- //:license is named by
# default_applicable_licenses across the tree -- so every load() in it is a
# load() every consumer pays for, and bazel_gazelle is a tool no consumer needs.
#
# The `# gazelle:` directives stay in the root BUILD, because gazelle reads
# repo-wide directives from the root package's BUILD file. Comments cost
# nothing to load.
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
