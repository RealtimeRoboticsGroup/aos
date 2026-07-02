load("@bazel_tools//tools/build_defs/repo:local.bzl", "local_repository")

def _mock_external_fbs_repo_impl(ctx):
    test_dir = ctx.path(Label("//aos/flatbuffers/test_dir:extensions.bzl")).dirname
    local_repository(
        name = "mock_external_fbs_repo",
        path = str(test_dir) + "/mock_external_fbs_repo",
    )

mock_external_fbs_repo_extension = module_extension(
    implementation = _mock_external_fbs_repo_impl,
)
