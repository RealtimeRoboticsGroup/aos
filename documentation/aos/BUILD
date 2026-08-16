load("@bazel_skylib//rules:copy_file.bzl", "copy_file")
load("//tools/build_rules:mkdocs.bzl", "mkdocs")

copy_file(
    name = "build_ping_pong_md",
    src = "//documentation/examples/ping_pong:build_ping_pong.md",
    out = "docs/build_ping_pong.md",
)

copy_file(
    name = "run_ping_pong_md",
    src = "//documentation/examples/ping_pong:run_ping_pong.md",
    out = "docs/run_ping_pong.md",
)

copy_file(
    name = "benchmarking_md",
    src = "//documentation/examples/starterd:benchmarking.md",
    out = "docs/benchmarking.md",
)

mkdocs(
    name = "docs",
    srcs = glob(["docs/*.md"]) + [
        ":benchmarking_md",
        ":build_ping_pong_md",
        ":run_ping_pong_md",
    ],
    config = "mkdocs.yaml",
)
