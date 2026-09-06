"""Third-party repos that must be private to AOS, fetched by one extension.

The 22 patches here change flatbuffers' behaviour rather than just its build:
force_defaults defaults to true (which changes the bytes the serializer
emits), FLATBUFFERS_ASSERT becomes ABSL_DCHECK, the verifier's max_tables
triples.  That must never resolve as anyone's `flatbuffers` module, and it
should not need a registry entry either.  An extension repo is private to AOS;
a consumer that wants the same flatbuffers AOS was built against reaches it
through AOS:

    aos_deps = use_extension("@aos//aos:extension.bzl", "dependencies")
    use_repo(aos_deps, "aos_flatbuffers")

Labels inside the fetched repo resolve through AOS's own dependency mapping,
so its BUILD files see the same @rules_cc, @rules_rust, @aspect_rules_ts, and
@flatbuffers_npm that AOS does.

foxglove_schemas rides along: its generated code must link against this
flatbuffers runtime (mixing it with a second runtime would be an ODR
violation), so it lives in the same extension. It is declared unconditionally
but fetched lazily, only when something builds one of its targets.
"""

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

_VERSION = "25.12.19"

# Shared with WORKSPACE mode, which applies the same patches to the same
# archive -- load this list there rather than repeating it.
FLATBUFFERS_PATCHES = [
    "01-verifier-max-tables-3m.patch",
    "02-cpp-has-clear-scalar-api.patch",
    "03-cpp-enum-ostream-and-reflection-include-remap.patch",
    "04-cpp-force-defaults-by-default.patch",
    "05-cpp-indeterminate-vector.patch",
    "06-cpp-default-allocator-zero-and-msan.patch",
    "07-cpp-numtostring-tochars.patch",
    "08-cpp-annotate-binary-string-and-fixes.patch",
    "09-flatc-python-import-prefix.patch",
    "10-idl-parser-deterministic-schema.patch",
    "11-codegen-wrap-in-namespace-object-suffix.patch",
    "12-rust-fully-qualified-name-and-follow-with.patch",
    "13-rust-runtime-allocations-and-apis.patch",
    "14-rust-codegen-warnings-and-bitflags.patch",
    "15-add-rust-build-bazel.patch",
    "16-build-bazel-aos-additions.patch",
    "17-reflection-explicit-ids-and-scoped-enums.patch",
    "18-ts-explicit-index-exports.patch",
    "19-python-fields-snake-case.patch",
    "20-cpp-absl-dcheck-assertions.patch",
    "21-python-object-api-types-other-packages.patch",
    "22-add-rust-lockfile.patch",
]

def _flatbuffers_impl(module_ctx):
    http_archive(
        name = "aos_flatbuffers",
        integrity = "sha256-+BwxYrEEb+i4S5oNvdOD4k/bz4hYO5y2Ao+Q0E2QaWo=",
        patch_args = ["-p1"],
        patches = [Label("//third_party/flatbuffers:" + p) for p in FLATBUFFERS_PATCHES],
        strip_prefix = "flatbuffers-" + _VERSION,
        urls = ["https://github.com/google/flatbuffers/archive/refs/tags/v%s.tar.gz" % _VERSION],
    )

    # Declared unconditionally: frc/vision's non-testonly targets name it, so
    # it has to resolve for consumers too. Declaring is free -- Bazel only
    # fetches a repo when something builds a target inside it.
    http_archive(
        name = "com_github_foxglove_schemas",
        build_file = Label("//third_party/foxglove_schemas:foxglove_schemas.BUILD"),
        integrity = "sha256-O3/7+jBCOJu1HpPqGiJake4FVm3HlthRd5FQW4vHU2s=",
        strip_prefix = "foxglove-sdk-sdk-v0.16.2",
        urls = ["https://github.com/foxglove/foxglove-sdk/archive/refs/tags/sdk/v0.16.2.tar.gz"],
    )

    return module_ctx.extension_metadata(reproducible = True)

dependencies = module_extension(
    implementation = _flatbuffers_impl,
)
