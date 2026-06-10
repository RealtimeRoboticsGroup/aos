load("@bazel_tools//tools/build_defs/repo:local.bzl", "local_repository")

def _mock_external_fbs_repo_impl(_ctx):
    local_repository(
        name = "mock_external_fbs_repo",
        path = "aos/flatbuffers/test_dir/mock_external_fbs_repo",
    )

mock_external_fbs_repo_extension = module_extension(
    implementation = _mock_external_fbs_repo_impl,
)
