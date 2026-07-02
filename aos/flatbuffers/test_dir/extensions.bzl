# Custom repository rule to dynamically link mock_external_fbs_repo.
# We use this instead of local_repository to avoid absolute path attributes
# being recorded in MODULE.bazel.lock, which makes the lockfile non-reproducible.
def _mock_external_fbs_repo_rule_impl(ctx):
    # Resolve the repository path dynamically using a local label reference.
    test_dir = ctx.path(Label("//aos/flatbuffers/test_dir:extensions.bzl")).dirname
    mock_repo_path = test_dir.get_child("mock_external_fbs_repo")

    # Symlink all elements from the target folder into the repository root.
    for entry in mock_repo_path.readdir():
        ctx.symlink(entry, entry.basename)

mock_external_fbs_repo_rule = repository_rule(
    implementation = _mock_external_fbs_repo_rule_impl,
)

def _mock_external_fbs_repo_impl(_ctx):
    mock_external_fbs_repo_rule(
        name = "mock_external_fbs_repo",
    )

mock_external_fbs_repo_extension = module_extension(
    implementation = _mock_external_fbs_repo_impl,
)
