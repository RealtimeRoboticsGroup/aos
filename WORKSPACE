workspace(name = "aos")

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

http_archive(
    name = "rules_cc",
    sha256 = "283fa1cdaaf172337898749cf4b9b1ef5ea269da59540954e51fba0e7b8f277a",
    strip_prefix = "rules_cc-0.2.17",
    url = "https://github.com/bazelbuild/rules_cc/releases/download/0.2.17/rules_cc-0.2.17.tar.gz",
)

http_archive(
    name = "rules_java",
    sha256 = "6ef26d4f978e8b4cf5ce1d47532d70cb62cd18431227a1c8007c8f7843243c06",
    urls = [
        "https://github.com/bazelbuild/rules_java/releases/download/9.3.0/rules_java-9.3.0.tar.gz",
    ],
)

# Keep WORKSPACE mode compatible with the bzlmod setup in MODULE.bazel.
# This matches the git_override commit and local patches used there.
http_archive(
    name = "rules_cuda",
    sha256 = "056f44415230fb063a090a9d0607d4346167d7c68d6bff4709520b132a8a8fe4",
    strip_prefix = "rules_cuda-345dd02d2b64b8a6c138800da3180c9278417ec2",
    urls = [
        "https://github.com/bazel-contrib/rules_cuda/archive/345dd02d2b64b8a6c138800da3180c9278417ec2.tar.gz",
    ],
)

load("@rules_java//toolchains:remote_java_repository.bzl", "remote_java_repository")
load("//tools/ci:repo_defs.bzl", "ci_configure")

ci_configure(name = "ci_configure")

load("@ci_configure//:ci.bzl", "RUNNING_IN_CI")
load("//:repositories.bzl", "aos_repositories", "frc_repositories")

# Keep WORKSPACE mode in sync with the archive_override commit in MODULE.bazel.
http_archive(
    name = "toolchains_llvm",
    sha256 = "8cafb7be80e2ccf0651e80b00de8072b498c5bd096f2227787441c9599b78d66",
    strip_prefix = "toolchains_llvm-1b9b565ac017dd69932ca5a41c6a1f5497b1eb21",
    url = "https://github.com/bazel-contrib/toolchains_llvm/archive/1b9b565ac017dd69932ca5a41c6a1f5497b1eb21.tar.gz",
)

local_repository(
    name = "com_github_wpilibsuite_allwpilib",
    path = "third_party/allwpilib",
)

http_archive(
    name = "bazel_features",
    sha256 = "07271d0f6b12633777b69020c4cb1eb67b1939c0cf84bb3944dc85cc250c0c01",
    strip_prefix = "bazel_features-1.38.0",
    url = "https://github.com/bazel-contrib/bazel_features/releases/download/v1.38.0/bazel_features-v1.38.0.tar.gz",
)

# toolchains_llvm now loads version helpers from @helly25_bzl in WORKSPACE
# mode. Declare it here so loading @toolchains_llvm//toolchain:rules.bzl works
# with --enable_workspace.
http_archive(
    name = "helly25_bzl",
    sha256 = "8846d5363ed05dfe242af692759c9b7439c1b7ce47b9720c3338e254651cbe99",
    strip_prefix = "bzl-0.4.3",
    url = "https://github.com/helly25/bzl/releases/download/0.4.3/bzl-0.4.3.tar.gz",
)

load("@bazel_features//:deps.bzl", "bazel_features_deps")

bazel_features_deps()

aos_repositories()

load("@bazel_tools//tools/build_defs/repo:utils.bzl", "maybe")
load("@platforms//host:extension.bzl", "host_platform_repo")

maybe(
    host_platform_repo,
    name = "host_platform",
)

load("@buildifier_prebuilt//:deps.bzl", "buildifier_prebuilt_deps")

buildifier_prebuilt_deps()

load("@buildifier_prebuilt//:defs.bzl", "buildifier_prebuilt_register_toolchains")

buildifier_prebuilt_register_toolchains()

frc_repositories()

load("@rules_cc//cc:extensions.bzl", "compatibility_proxy_repo")

compatibility_proxy_repo()

load("//:repositories2.bzl", "dependencies_phase1")

dependencies_phase1()

load(
    "@aos//tools/python:package_annotations.bzl",
    PYTHON_ANNOTATIONS = "ANNOTATIONS",
)
load("@rules_multitool//multitool:multitool.bzl", "multitool")
load("@rules_python//python:pip.bzl", "pip_parse")

pip_parse(
    name = "pip_deps",
    timeout = 1800,
    annotations = PYTHON_ANNOTATIONS,
    download_only = RUNNING_IN_CI,
    enable_implicit_namespace_pkgs = True,
    extra_pip_args = [
        # The https://realtimeroboticsgroup.org mirror can be slower than the
        # upstream index. Bump the timeout to avoid issues.
        "--timeout=1800",
    ] + ([
        "--index-url=https://realtimeroboticsgroup.org/build-dependencies/wheelhouse/simple",
        # Ignore SSL for now
        "--trusted-host=realtimeroboticsgroup.org",
    ] if RUNNING_IN_CI else [
        "--index-url=https://pypi.org/simple",
        "--extra-index-url=https://realtimeroboticsgroup.org/build-dependencies/wheelhouse/simple",
        "--prefer-binary",
    ]),
    python_interpreter_target = "@python_3_10_host//:python",
    requirements_lock = "//tools/python:requirements.lock.txt",
)

# Load the starlark macro which will define your dependencies.
load(
    "@pip_deps//:requirements.bzl",
    install_pip_deps = "install_deps",
)

install_pip_deps(
    patch_spec = {
        "matplotlib": {
            patch: json.encode({"patch_strip": 2})
            for patch in [
                "//third_party:python/matplotlib/init.patch",
            ]
        },
        "pygobject": {
            patch: json.encode({"patch_strip": 2})
            for patch in [
                "//third_party:python/pygobject/init.patch",
            ]
        },
    },
)

# As long as we're using WORKSPACE, this will only work if uv is the only thing
# using the multitool repo name. Otherwise, we'll have to patch it.
multitool(
    name = "multitool",
    lockfile = "@rules_uv//uv/private:uv.lock.json",
)

load("@multitool//:tools.bzl", "register_tools")

register_tools()

load("//debian:packages.bzl", "generate_repositories_for_debs")
load(
    "//debian:phoenix6.bzl",
    phoenix6_debs = "files",
)

generate_repositories_for_debs(phoenix6_debs)

# Register @llvm_distributions_data before loading rules.bzl: rules.bzl ->
# repo.bzl -> llvm_distributions.bzl transitively loads
# @llvm_distributions_data//:data.bzl, so the repo must exist first. In bzlmod
# this is done by the llvm_distributions module extension instead.
load("@toolchains_llvm//toolchain:setup_distributions.bzl", "setup_llvm_distributions")

setup_llvm_distributions()

load("@toolchains_llvm//toolchain:rules.bzl", "llvm", "llvm_toolchain")

llvm_version = "21.1.8"

llvm_alternative_sources = [
    "https://github.com/RealtimeRoboticsGroup/toolchains/releases/download/{llvm_version}/{basename}",
]

llvm_x86_64_basename = "clang+llvm-%s-x86_64-linux-gnu-ubuntu-22.04.tar.zst" % llvm_version

llvm_aarch64_basename = "clang+llvm-%s-aarch64-linux-gnu.tar.zst" % llvm_version

llvm_extra_distributions = {
    llvm_x86_64_basename: "3e9ab559182f45143c36d761fcee6818935187a8f54354ecb043643b642769f0",
    llvm_aarch64_basename: "68172a4b557ab3ae50fcc27d3e706d066ea5891933e50f6fb97d6ca9f7e2beb4",
}

llvm(
    name = "llvm_k8",
    alternative_llvm_sources = llvm_alternative_sources,
    distribution = llvm_x86_64_basename,
    extra_llvm_distributions = llvm_extra_distributions,
    # MemorySanitizer-instrumented libc++ overlay, swapped in for builds with
    # `--features=msan` (see `build:msan` in .bazelrc). Same LLVM version as the
    # base toolchain so it is ABI-compatible with clang 21.1.8.
    libcxx_sha256 = "56e33981bb71aba8704026534093f81ba9783cff0bf701681f1781d2ecb50a87",
    libcxx_url = "https://github.com/RealtimeRoboticsGroup/toolchains/releases/download/21.1.8/libcxx-msan-21.1.8-x86_64-linux-gnu-ubuntu-22.04.tar.zst",
    llvm_versions = {"": llvm_version},
)

llvm(
    name = "llvm_aarch64",
    alternative_llvm_sources = llvm_alternative_sources,
    distribution = llvm_aarch64_basename,
    extra_llvm_distributions = llvm_extra_distributions,
    # MemorySanitizer-instrumented libc++ overlay, swapped in for builds with
    # `--features=msan` (see `build:msan` in .bazelrc). Same LLVM version as the
    # base toolchain so it is ABI-compatible with clang 21.1.8.
    libcxx_sha256 = "3bdcaf1d171ea3e15b9cef3229db0476cc8d333a32c32df9157293328f251a68",
    libcxx_url = "https://github.com/RealtimeRoboticsGroup/toolchains/releases/download/21.1.8/libcxx-msan-21.1.8-aarch64-linux-gnu.tar.zst",
    llvm_versions = {"": llvm_version},
)

llvm_conly_flags = [
    "-std=gnu99",
]

llvm_extra_compile_flags = [
    "-D__STDC_FORMAT_MACROS",
    "-D__STDC_CONSTANT_MACROS",
    "-D__STDC_LIMIT_MACROS",
    "-D_FILE_OFFSET_BITS=64",
    "-fmessage-length=100",
    "-fmacro-backtrace-limit=0",
    "-ggdb3",
    # Too many core libraries have these right now.
    # TODO(austin): Turn this off later.
    "-Wno-deprecated-declarations",
    "-Wembedded-directive",
]

llvm_extra_compile_flags_aarch64 = llvm_extra_compile_flags + ["-march=armv8-a+crc"]

llvm_cxx_standard = "gnu++20"

llvm_toolchain(
    name = "llvm_toolchain",
    conly_flags = {
        "linux-aarch64": llvm_conly_flags,
        "linux-x86_64": llvm_conly_flags,
    },
    cxx_include_layout = {
        "linux-aarch64": "yocto",
    },
    cxx_standard = {
        "linux-aarch64": llvm_cxx_standard,
        "linux-x86_64": llvm_cxx_standard,
    },
    extra_compile_flags = {
        "linux-aarch64": llvm_extra_compile_flags_aarch64,
        "linux-x86_64": llvm_extra_compile_flags,
    },
    llvm_version = llvm_version,
    multiarch = {
        "linux-aarch64": "aarch64-oe4t-linux",
    },
    stdlib = {
        "linux-aarch64": "dynamic-stdc++-14.3.0",
        "linux-x86_64": "dynamic-stdc++-12",
    },
    sysroot = {
        "linux-aarch64": "@arm64_debian_sysroot//:sysroot_files",
        "linux-x86_64": "@amd64_debian_sysroot//:sysroot_files",
    },
    target_toolchain_roots = {
        "linux-aarch64": "@llvm_aarch64//",
        "linux-x86_64": "@llvm_k8//",
    },
    toolchain_roots = {
        "linux-aarch64": "@llvm_aarch64//",
        "linux-x86_64": "@llvm_k8//",
    },
)

load("@llvm_toolchain//:toolchains.bzl", "llvm_register_toolchains")

llvm_register_toolchains()

register_toolchains(
    "//tools/cpp:cc-toolchain-roborio",
    "//tools/cpp:cc-toolchain-cortex-m4f",
    "//tools/cpp:cc-toolchain-rp2040",
    "//tools/cpp:cc-toolchain-cortex-m4f-imu",
    # Find a good way to select between these two M4F toolchains.
    #"//tools/cpp:cc-toolchain-cortex-m4f-k22",
    "//tools/python:python_toolchain",
    "//tools/go:noop_go_toolchain",
    "//tools/rust:rust-toolchain-x86",
    "//tools/rust:rust-toolchain-armv7",
    "//tools/rust:rust-toolchain-arm64",
    # TODO(Brian): Make this work. See the comment on
    # //tools/platforms:linux_roborio for details.
    #"//tools/rust:rust-toolchain-roborio",
    "//tools/rust:noop_rust_toolchain",
    "//tools/ts:noop_node_toolchain",
    # CUDA toolchains for rules_cuda (workspace mode)
    "//tools/cuda:x86_64_clang_toolchain",
    "//tools/cuda:aarch64_clang_toolchain",
    "//tools/cuda:no_gpu_clang_toolchain",
)

# Java11 JDK.
remote_java_repository(
    name = "openjdk_linux_archive",
    prefix = "openjdk",
    sha256 = "60e65d32e38876f81ddb623e87ac26c820465b637e263e8bed1acdecb4ca9be2",
    strip_prefix = "zulu11.54.25-ca-jdk11.0.14.1-linux_x64",
    target_compatible_with = [
        "@platforms//cpu:x86_64",
        "@platforms//os:linux",
    ],
    urls = [
        "https://realtimeroboticsgroup.org/build-dependencies/zulu11.54.25-ca-jdk11.0.14.1-linux_x64.tar.gz",
    ],
    version = "11",
)

remote_java_repository(
    name = "openjdk_linux_archive_aarch64",
    prefix = "openjdk",
    sha256 = "b0fb0bc303bb05b5042ef3d0939b9489f4a49a13a2d1c8f03c5d8ab23099454d",
    strip_prefix = "zulu11.54.25-ca-jdk11.0.14.1-linux_aarch64",
    target_compatible_with = [
        "@platforms//cpu:aarch64",
        "@platforms//os:linux",
    ],
    urls = [
        "https://realtimeroboticsgroup.org/build-dependencies/zulu11.54.25-ca-jdk11.0.14.1-linux_aarch64.tar.gz",
    ],
    version = "11",
)

load("@aspect_rules_js//js:repositories.bzl", "rules_js_dependencies")
load("@aspect_rules_rollup//rollup:dependencies.bzl", "rules_rollup_dependencies")

rules_rollup_dependencies()

rules_js_dependencies()

load("@aspect_rules_esbuild//esbuild:dependencies.bzl", "rules_esbuild_dependencies")
load("@aspect_rules_js//js:toolchains.bzl", "DEFAULT_NODE_VERSION", "rules_js_register_toolchains")
load("@aspect_rules_terser//terser:dependencies.bzl", "rules_terser_dependencies")
load("@aspect_rules_ts//ts:repositories.bzl", "rules_ts_dependencies")

rules_esbuild_dependencies()

rules_js_register_toolchains(node_version = DEFAULT_NODE_VERSION)

rules_terser_dependencies()

rules_ts_dependencies(
    ts_version_from = "//:package.json",
)

load("@aspect_rules_js//npm:repositories.bzl", "npm_translate_lock")

npm_translate_lock(
    name = "npm",
    data = [
        "//aos/analysis/foxglove_extension:package.json",
        "//control_loops/swerve/spline_ui/www:package.json",
        "@//:package.json",
        "@//:pnpm-workspace.yaml",
    ],

    # Running lifecycle hooks on npm package fsevents@2.3.2 fails in a dramatic way:
    # ```
    # SyntaxError: Unexpected strict mode reserved word
    # at ESMLoader.moduleStrategy (node:internal/modules/esm/translators:117:18)
    # at ESMLoader.moduleProvider (node:internal/modules/esm/loader:337:14)
    # at async link (node:internal/modules/esm/module_job:70:21)
    # ```
    lifecycle_hooks_no_sandbox = False,
    npmrc = "//:.npmrc",
    pnpm_lock = "//:pnpm-lock.yaml",
    quiet = False,
    update_pnpm_lock = False,
    verify_node_modules_ignored = "//:.bazelignore",
)

load("@aspect_rules_esbuild//esbuild:repositories.bzl", "LATEST_ESBUILD_VERSION", "esbuild_register_toolchains")

esbuild_register_toolchains(
    name = "esbuild",
    esbuild_version = LATEST_ESBUILD_VERSION,
)

load("@npm//:repositories.bzl", "npm_repositories")

npm_repositories()

load("@aspect_rules_cypress//cypress:dependencies.bzl", "rules_cypress_dependencies")
load("@aspect_rules_cypress//cypress:repositories.bzl", "cypress_register_toolchains")

rules_cypress_dependencies()

cypress_register_toolchains(
    name = "cypress",
    cypress_version = "13.3.1",
)

# Flatbuffers
local_repository(
    name = "com_github_google_flatbuffers",
    path = "third_party/flatbuffers/",
)

load("@com_github_google_flatbuffers//ts:repositories.bzl", "flatbuffers_npm")

flatbuffers_npm(
    name = "flatbuffers_npm",
)

load("@flatbuffers_npm//:repositories.bzl", fbs_npm_repositories = "npm_repositories")

fbs_npm_repositories()

http_archive(
    name = "rules_rust",
    integrity = "sha256-w4tiLybzXDRzgQDibReT/yUolzgVRkZ7IticnU2L/VA=",
    urls = ["https://github.com/bazelbuild/rules_rust/releases/download/0.63.0/rules_rust-0.63.0.tar.gz"],
)

load("@rules_rust//rust:repositories.bzl", "rust_analyzer_toolchain_repository", "rust_repository_set")

RUST_VERSION = "1.81.0"

rust_repository_set(
    name = "rust",
    allocator_library = "@//tools/rust:forward_allocator",
    edition = "2021",
    exec_triple = "x86_64-unknown-linux-gnu",
    extra_target_triples = [
        "arm-unknown-linux-gnueabi",
        "armv7-unknown-linux-gnueabihf",
        "aarch64-unknown-linux-gnu",
    ],
    register_toolchain = False,
    rustfmt_version = RUST_VERSION,
    versions = [RUST_VERSION],
)

load("@rules_rust//crate_universe:repositories.bzl", "crate_universe_dependencies")

crate_universe_dependencies()

load("@rules_rust//crate_universe:defs.bzl", "crate", "crates_repository")

# Run `CARGO_BAZEL_REPIN=1 bazel sync --only=crate_index` to update the lock file
# after adding or removing any dependencies.
crates_repository(
    name = "crate_index",
    annotations = {
        "cxx": [
            crate.annotation(
                additive_build_file = "@aos//third_party/cargo:cxx/include.BUILD.bazel",
                extra_aliased_targets = {"cxx_cc": "cxx_cc"},
                gen_build_script = False,
            ),
        ],
        "link-cplusplus": [
            # Bazel toolchains take care of linking the C++ standard library, so don't add
            # an extra flag via Rust by enabling the `nothing` feature. I'm not even sure
            # it would end up on the link command line, but this crate's build.rs attempts
            # to find a C++ compiler itself otherwise which definitely doesn't work.
            crate.annotation(
                crate_features = ["nothing"],
            ),
        ],
        "log": [
            crate.annotation(
                rustc_flags = ["--cfg=atomic_cas"],
            ),
        ],
    },
    cargo_lockfile = "//:Cargo.lock",
    lockfile = "//:Cargo.Bazel.Workspace.lock",
    manifests = [
        "//:Cargo.toml",
        "//third_party/autocxx:Cargo.toml",
        "//third_party/autocxx:engine/Cargo.toml",
        "//third_party/autocxx:parser/Cargo.toml",
        "//third_party/autocxx:gen/cmd/Cargo.toml",
        "//third_party/autocxx:macro/Cargo.toml",
        "//third_party/autocxx:integration-tests/Cargo.toml",
    ],
    rust_toolchain_cargo_template = "@rust__{triple}__{channel}_tools//:bin/{tool}",
    rust_toolchain_rustc_template = "@rust__{triple}__{channel}_tools//:bin/{tool}",
    rust_version = RUST_VERSION,
    supported_platform_triples = [
        "x86_64-unknown-linux-gnu",
        "arm-unknown-linux-gnueabi",
        "armv7-unknown-linux-gnueabihf",
        "aarch64-unknown-linux-gnu",
    ],
)

load("@crate_index//:defs.bzl", "crate_repositories")

crate_repositories()

load("@rules_rust//tools/rust_analyzer:deps.bzl", "rust_analyzer_dependencies")

rust_analyzer_dependencies()

register_toolchains(rust_analyzer_toolchain_repository(
    name = "rust_analyzer_toolchain",
    version = RUST_VERSION,
))

http_archive(
    name = "cxxbridge-cmd",
    build_file = "//third_party/cargo:cxxbridge-cmd/include.BUILD.bazel",
    sha256 = "df13eece12ed9e7bd4fb071a6af4c44421bb9024d339d029f5333bcdaca00000",
    strip_prefix = "cxxbridge-cmd-1.0.100",
    type = "tar.gz",
    urls = ["https://crates.io/api/v1/crates/cxxbridge-cmd/1.0.100/download"],
)

crates_repository(
    name = "cxxbridge_cmd_deps",
    cargo_lockfile = "//third_party/cargo:cxxbridge-cmd/Cargo.lock",
    lockfile = "//third_party/cargo:cxxbridge-cmd/Cargo.Bazel.Workspace.lock",
    manifests = ["@cxxbridge-cmd//:Cargo.toml"],
    rust_toolchain_cargo_template = "@rust__{triple}__{channel}_tools//:bin/{tool}",
    rust_toolchain_rustc_template = "@rust__{triple}__{channel}_tools//:bin/{tool}",
    rust_version = "1.81.0",
    supported_platform_triples = [
        "x86_64-unknown-linux-gnu",
        "arm-unknown-linux-gnueabi",
        "armv7-unknown-linux-gnueabihf",
        "aarch64-unknown-linux-gnu",
    ],
)

load("@cxxbridge_cmd_deps//:defs.bzl", cxxbridge_cmd_deps = "crate_repositories")

cxxbridge_cmd_deps()

http_archive(
    name = "io_bazel_rules_go",
    patch_args = [
        "-p1",
    ],
    patches = [
        "@aos//third_party:rules_go/0001-Disable-warnings-for-external-repositories.patch",
    ],
    sha256 = "763f4a3f6b03469fdb00a77a333dd0b5546d3ee1fa29db373128c08fee73e0e8",
    urls = [
        "https://github.com/bazelbuild/rules_go/releases/download/v0.61.1/rules_go-v0.61.1.zip",
    ],
)

http_archive(
    name = "bazel_gazelle",
    patch_args = [
        "-p1",
    ],
    patches = [
        "@aos//third_party:bazel-gazelle/0001-Fix-visibility-of-gazelle-runner.patch",
    ],
    sha256 = "49d9eba309b0b695824ff417d734242824ad9ab5edb56063b9d3400df1a61a56",
    urls = [
        "https://mirror.bazel.build/github.com/bazel-contrib/bazel-gazelle/releases/download/v0.51.3/bazel-gazelle-v0.51.3.tar.gz",
        "https://github.com/bazel-contrib/bazel-gazelle/releases/download/v0.51.3/bazel-gazelle-v0.51.3.tar.gz",
    ],
)

load("@io_bazel_rules_go//go:deps.bzl", "go_download_sdk", "go_register_toolchains", "go_rules_dependencies")

go_download_sdk(
    name = "go_sdk",
    goarch = "amd64",
    goos = "linux",
    sdks = {
        "darwin_amd64": ("go1.24.13.darwin-amd64.tar.gz", "6cc6549b06725220b342b740497ffd24e0ebdcef75781a77931ca199f46ad781"),
        # Pulled from https://go.dev/dl/ to avoid the external dependency.
        "linux_amd64": ("go1.24.13.linux-amd64.tar.gz", "1fc94b57134d51669c72173ad5d49fd62afb0f1db9bf3f798fd98ee423f8d730"),
    },
    version = "1.24.13",
)

go_rules_dependencies()

go_register_toolchains()

load("@bazel_gazelle//:deps.bzl", "gazelle_dependencies")
load("//:go_deps.bzl", "go_dependencies")
load("//tools/go:mirrored_go_deps.bzl", "mirrored_go_dependencies")

mirrored_go_dependencies()

# gazelle:repository_macro go_deps.bzl%go_dependencies
go_dependencies()

gazelle_dependencies()

load("@com_github_grpc_grpc//bazel:grpc_deps.bzl", "grpc_deps")

grpc_deps()

load("@com_github_grpc_grpc//bazel:grpc_extra_deps.bzl", "grpc_extra_deps")

grpc_extra_deps()

load("@hedron_compile_commands//:workspace_setup.bzl", "hedron_compile_commands_setup")

hedron_compile_commands_setup()

load("@com_github_storypku_bazel_iwyu//bazel:dependencies.bzl", "bazel_iwyu_dependencies")

bazel_iwyu_dependencies()

local_repository(
    name = "mock_external_fbs_repo",
    path = "aos/flatbuffers/test_dir/mock_external_fbs_repo",
)
