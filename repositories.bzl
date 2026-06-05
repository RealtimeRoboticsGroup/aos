load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive", "http_file")
load("//:repositories_internal.bzl", "april_tag_test_image_repo", "apriltag_test_bfbs_images_repo", "arm_frc_linux_gnueabi_repo_repo", "calibrate_multi_cameras_data_repo", "com_github_foxglove_mcap_mcap_repo", "com_github_nvidia_cccl_repo", "coral_image_thriftycam_2025_repo", "drivetrain_replay_repo", "frc2026_field_map_welded_repo", "intrinsic_calibration_test_images_repo", "orin_capture_24_04_repo", "orin_capture_24_04_side_repo", "orin_image_apriltag_repo", "orin_large_image_apriltag_repo", "superstructure_replay_repo")

def _compat_repository_impl(ctx):
    ctx.file("BUILD", ctx.attr.build_file_content)
    ctx.file("WORKSPACE", "workspace(name = \"{}\")".format(ctx.name))

compat_repository = repository_rule(
    implementation = _compat_repository_impl,
    attrs = {
        "build_file_content": attr.string(),
    },
)

def aos_repositories(prefix = ""):
    http_archive(
        name = "platforms",
        sha256 = "dbad4a23abcca6171e47b79edc53bd6a41067a3b75f9e8b104656b459ff25046",
        urls = [
            "https://github.com/bazelbuild/platforms/releases/download/1.1.0/platforms-1.1.0.tar.gz",
        ],
    )

    http_archive(
        name = "buildifier_prebuilt",
        sha256 = "7f85b688a4b558e2d9099340cfb510ba7179f829454fba842370bccffb67d6cc",
        strip_prefix = "buildifier-prebuilt-7.3.1",
        urls = [
            "https://github.com/keith/buildifier-prebuilt/archive/7.3.1.tar.gz",
        ],
    )

    compat_repository(
        name = "aos_rust_host_tools",
        build_file_content = """
alias(
    name = "rustfmt",
    actual = "@rust__x86_64-unknown-linux-gnu__stable_tools//:rustfmt",
    visibility = ["//visibility:public"],
)
""",
    )

    http_archive(
        name = "bazel_skylib",
        sha256 = "51b5105a760b353773f904d2bbc5e664d0987fbaf22265164de65d43e910d8ac",
        urls = [
            "https://mirror.bazel.build/github.com/bazelbuild/bazel-skylib/releases/download/1.8.1/bazel-skylib-1.8.1.tar.gz",
            "https://github.com/bazelbuild/bazel-skylib/releases/download/1.8.1/bazel-skylib-1.8.1.tar.gz",
        ],
    )

    http_archive(
        name = "aspect_bazel_lib",
        sha256 = "40ba9d0f62deac87195723f0f891a9803a7b720d7b89206981ca5570ef9df15b",
        strip_prefix = "bazel-lib-2.14.0",
        url = "https://github.com/bazel-contrib/bazel-lib/releases/download/v2.14.0/bazel-lib-v2.14.0.tar.gz",
    )

    http_archive(
        name = "com_google_absl",
        patch_args = ["-p1"],
        patches = [
            "@aos//third_party/abseil:0001-Add-hooks-for-using-abseil-with-AOS.patch",
            "@aos//third_party/abseil:0002-Suppress-the-stack-trace-on-SIGABRT.patch",
            "@aos//third_party/abseil:0004-Remove-relocatability-test-that-is-no-longer-useful.patch",
        ],
        repo_mapping = {
            "@google_benchmark": "@com_github_google_benchmark",
            "@googletest": "@com_google_googletest",
        },
        sha256 = "9b7a064305e9fd94d124ffa6cc358592eb42b5da588fb4e07d09254aa40086db",
        strip_prefix = "abseil-cpp-20250512.1",
        url = "https://github.com/abseil/abseil-cpp/archive/refs/tags/20250512.1.tar.gz",
    )

    http_archive(
        name = "com_google_protobuf",
        repo_mapping = {"@abseil-cpp": "@com_google_absl"},
        sha256 = "12bfd76d27b9ac3d65c00966901609e020481b9474ef75c7ff4601ac06fa0b82",
        strip_prefix = "protobuf-31.1",
        url = "https://github.com/protocolbuffers/protobuf/releases/download/v31.1/protobuf-31.1.tar.gz",
    )

    http_archive(
        name = "rules_python",
        patch_args = ["-p1"],
        patches = [
            "@aos//third_party:rules_python/0001-Allow-WORKSPACE-users-to-patch-wheels.patch",
            "@aos//third_party:rules_python/0002-Allow-users-to-inject-extra-deps.patch",
        ],
        sha256 = "0a1cefefb4a7b550fb0b43f54df67d6da95b7ba352637669e46c987f69986f6a",
        strip_prefix = "rules_python-1.5.3",
        url = "https://github.com/bazel-contrib/rules_python/releases/download/1.5.3/rules_python-1.5.3.tar.gz",
    )

    http_archive(
        name = "rules_uv",
        sha256 = "bfbe18fed6242e47f4b22918f43abdc0e274d07c3174d44ef1d29f7aa3d3bb4c",
        strip_prefix = "rules_uv-0.75.0",
        url = "https://github.com/theoremlp/rules_uv/releases/download/v0.75.0/rules_uv-0.75.0.tar.gz",
    )

    http_archive(
        name = "rules_multitool",
        sha256 = "1037e1b11d42ee56751449b3b1e995ca7b9af76d7665dfefcc7112919551d45b",
        strip_prefix = "rules_multitool-1.4.0",
        url = "https://github.com/theoremlp/rules_multitool/releases/download/v1.4.0/rules_multitool-1.4.0.tar.gz",
    )

    http_archive(
        name = "rules_pkg",
        sha256 = "b7215c636f22c1849f1c3142c72f4b954bb12bb8dcf3cbe229ae6e69cc6479db",
        url = "https://github.com/bazelbuild/rules_pkg/releases/download/1.1.0/rules_pkg-1.1.0.tar.gz",
    )

    # Download toolchains
    http_archive(
        name = "rules_bzlmodrio_toolchains",
        sha256 = "102b4507628e9724b0c1e441727762c344e40170f65ac60516168178ea33a89a",
        url = "https://github.com/wpilibsuite/rules_bzlmodrio_toolchains/releases/download/2025-1.bcr6/rules_bzlmodrio_toolchains-2025-1.bcr6.tar.gz",
    )

    http_archive(
        name = "bzlmodrio-ni",
        sha256 = "fff62c3cb3e83f9a0d0a01f1739477c9ca5e9a6fac05be1ad59dafcd385801f7",
        url = "https://github.com/wpilibsuite/bzlmodRio-ni/releases/download/2025.2.0/bzlmodRio-ni-2025.2.0.tar.gz",
    )

    http_archive(
        name = "RangeHTTPServer",
        sha256 = "98a8e4980f91d048dc9159cfc5f115280d0b5ec59a5b01df0422b887212fa4f0",
        strip_prefix = "RangeHTTPServer-9070394508a135789238a33259793f3c6f3c127a",
        url = "https://github.com/jkuszmaul/RangeHTTPServer/archive/9070394508a135789238a33259793f3c6f3c127a.zip",
    )

    arm_frc_linux_gnueabi_repo_repo()

    # The main partition packaged with //compilers/buildify_yocto_image.py
    # Packaging the yocto image built from https://github.com/frc4646/meta-frc4646
    # To rebuild, follow the instructions in meta-frc4646, then, cd compilers and
    # run buildify_yocto_image.py /path/to/git/checkout/meta-frc4646
    http_archive(
        name = "arm64_debian_sysroot",
        build_file = "//:registry/modules/arm64_debian_sysroot/2025.10.25/overlay/BUILD.bazel",
        sha256 = "60982dba37854900206e61fa572c6510197c531db0f5f1b60ce0623803e5cb9a",
        url = "https://realtimeroboticsgroup.org/build-dependencies/2025-10-25-walnascar-arm64-nvidia-rootfs.tar.zst",
    )

    # Sysroot generated using //frc/amd64/build_rootfs.py
    http_archive(
        name = "amd64_debian_sysroot",
        build_file = "//:registry/modules/amd64_debian_sysroot/2025.04.20/overlay/BUILD.bazel",
        sha256 = "e94dec03e19d88cd428964f1e4a430e6bc4a2dd2f4f7342f56b75efa9c75a761",
        url = "https://realtimeroboticsgroup.org/build-dependencies/2025-04-20-bookworm-amd64-nvidia-rootfs.tar.zst",
    )

    # Originally from: https://developer.nvidia.com/downloads/compute/machine-learning/tensorrt/10.9.0/tars/TensorRT-10.9.0.34.Linux.x86_64-gnu.cuda-11.8.tar.gz
    # Recompressed for faster extraction.
    http_archive(
        name = "amd64_tensorrt",
        build_file = "@aos//:compilers/tensorrt.BUILD",
        sha256 = "5b5b828be725077d13a23d296a5ae56ce42f785ad8258dc79d34f326b34cc783",
        strip_prefix = "TensorRT-10.9.0.34",
        url = "https://realtimeroboticsgroup.org/build-dependencies/developer.nvidia.com/downloads/compute/machine-learning/tensorrt/10.9.0/tars/TensorRT-10.9.0.34.Linux.x86_64-gnu.cuda-11.8.tar.zst",
    )

    april_tag_test_image_repo()

    # Downloaded from
    # From https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads
    http_archive(
        name = "gcc_arm_none_eabi",
        build_file = "@aos//:compilers/gcc_arm_none_eabi.BUILD",
        sha256 = "6cd1bbc1d9ae57312bcd169ae283153a9572bd6a8e4eeae2fedfbc33b115fdbb",
        strip_prefix = "arm-gnu-toolchain-13.2.Rel1-x86_64-arm-none-eabi",
        url = "https://developer.arm.com/-/media/Files/downloads/gnu/13.2.rel1/binrel/arm-gnu-toolchain-13.2.rel1-x86_64-arm-none-eabi.tar.xz",
    )

    native.local_repository(
        name = "eigen",
        path = prefix + "third_party/eigen",
    )

    native.local_repository(
        name = "gmp",
        path = prefix + "third_party/gmp",
    )

    native.local_repository(
        name = "glib",
        path = prefix + "third_party/glib",
    )

    native.local_repository(
        name = "xz",
        path = prefix + "third_party/xz",
    )

    # This one is tricky to get an archive because it has recursive submodules. These semi-automated steps do work though:
    # git clone -b 1.11.321 --recurse-submodules --depth=1 https://github.com/aws/aws-sdk-cpp
    # cd aws-sdk-cpp
    # echo bsdtar -a -cf aws_sdk-version.tar.gz --ignore-zeros @\<\(git archive HEAD\) $(git submodule foreach --recursive --quiet 'echo @\<\(cd $displaypath \&\& git archive HEAD --prefix=$displaypath/\)')
    # Now run the command that printed, and the output will be at aws_sdk-version.tar.gz.
    http_archive(
        name = "aws_sdk",
        build_file = "@aos//debian:aws_sdk.BUILD",
        sha256 = "08856b91139d209f7423e60dd8f74a14ab6d053ca40088fcb42fd02484003e95",
        url = "https://realtimeroboticsgroup.org/build-dependencies/aws_sdk-1.11.321.tar.gz",
    )

    http_archive(
        name = "snappy",
        sha256 = "90f74bc1fbf78a6c56b3c4a082a05103b3a56bb17bca1a27e052ea11723292dc",
        strip_prefix = "snappy-1.2.2",
        url = "https://github.com/google/snappy/archive/refs/tags/1.2.2.tar.gz",
    )

    http_archive(
        name = "rawrtc_re",
        patch_args = ["-p1"],
        patches = ["@aos//third_party:rawrtc/rawrtc_re.patch"],
        sha256 = "39b31ae8fb98d3c1f405f7d55dc272af1f0f5ba1a6f2381ff88e917d0d7672c9",
        strip_prefix = "re-9384f3a5f38a03c871270fda566045b3bf57bbee",
        url = "https://github.com/rawrtc/re/archive/9384f3a5f38a03c871270fda566045b3bf57bbee.tar.gz",
    )

    http_archive(
        name = "rawrtc_rew",
        patch_args = ["-p1"],
        patches = ["@aos//third_party:rawrtc/rawrtc_rew.patch"],
        sha256 = "03fe0408b3bdc2e820c0a29e40e3d769028882c7eb7e30de6b7fe61e42a2ff6e",
        strip_prefix = "rew-24c91fd839b40b11f727c902fa46d20874da33fb",
        url = "https://github.com/rawrtc/rew/archive/24c91fd839b40b11f727c902fa46d20874da33fb.tar.gz",
    )

    http_archive(
        name = "rawrtc_usrsctp",
        patch_args = ["-p1"],
        patches = ["@aos//third_party:rawrtc/rawrtc_usrsctp.patch"],
        sha256 = "374ef5acacc981a8b27b4fe957f081c2e44b45fdb55e652c22ed9012d9ccaf6a",
        strip_prefix = "usrsctp-bd1a92db338ba1e57453637959a127032bb566ff",
        url = "https://github.com/rawrtc/usrsctp/archive/bd1a92db338ba1e57453637959a127032bb566ff.tar.gz",
    )

    http_archive(
        name = "rawrtc_common",
        patch_args = ["-p1"],
        patches = ["@aos//third_party:rawrtc/rawrtc_rawrtc_common.patch"],
        sha256 = "8ad48a7231aa00d2218392b0fc23e17d34a161f2cc5347f7f38a37bb7e6271a5",
        strip_prefix = "rawrtc-common-aff7a3a3b9bbf49f7d2fc8b123edd301825b3e1c",
        url = "https://github.com/rawrtc/rawrtc-common/archive/aff7a3a3b9bbf49f7d2fc8b123edd301825b3e1c.tar.gz",
    )

    http_archive(
        name = "rawrtc_data_channel",
        patch_args = ["-p1"],
        patches = ["@aos//third_party:rawrtc/rawrtc_rawrtc_data_channel.patch"],
        sha256 = "0c5bfed79faf5dec5a7de7669156e5d9dacbf425ad495127bab52f28b56afaa8",
        strip_prefix = "rawrtc-data-channel-7b1b8d57c6d07da18cc0de8bbca8cc5e8bd06eae",
        url = "https://github.com/rawrtc/rawrtc-data-channel/archive/7b1b8d57c6d07da18cc0de8bbca8cc5e8bd06eae.tar.gz",
    )

    http_archive(
        name = "rawrtc",
        patch_args = ["-p1"],
        patches = ["@aos//third_party:rawrtc/rawrtc_rawrtc.patch"],
        sha256 = "ba449026f691ce57cd9a685b1a16d1bbb88d63d8b5eac38406ed7b53d63c9d07",
        strip_prefix = "rawrtc-aa3ae4b247275cc6e69c30613b3a4ba7fdc82d1b",
        url = "https://github.com/rawrtc/rawrtc/archive/aa3ae4b247275cc6e69c30613b3a4ba7fdc82d1b.tar.gz",
    )

    http_file(
        name = "sample_logfile",
        downloaded_file_path = "log.fbs",
        sha256 = "45d1d19fb82786c476d3f21a8d62742abaeeedf4c16a00ec37ae350dcb61f1fc",
        urls = ["https://realtimeroboticsgroup.org/build-dependencies/small_sample_logfile2.fbs"],
    )

    com_github_foxglove_mcap_mcap_repo()

    http_archive(
        name = "com_google_googletest",
        sha256 = "8ad598c73ad796e0d8280b082cebd82a630d73e73cd3c70057938a6501bba5d7",
        strip_prefix = "googletest-1.14.0",
        urls = ["https://github.com/google/googletest/archive/refs/tags/v1.14.0.tar.gz"],
    )

    http_archive(
        name = "com_github_google_benchmark",
        patch_args = ["-p1"],
        patches = ["@aos//third_party/google-benchmark:benchmark.patch"],
        sha256 = "3e7059b6b11fb1bbe28e33e02519398ca94c1818874ebed18e504dc6f709be45",
        strip_prefix = "benchmark-1.8.4",
        urls = ["https://github.com/google/benchmark/archive/refs/tags/v1.8.4.tar.gz"],
    )

    http_archive(
        name = "ceres-solver",
        patch_args = ["-p1"],
        patches = ["@aos//third_party:ceres.patch"],
        sha256 = "5fef6cd0ed744a09e20d1c341a15b0f94ed0c8df43537e198a869e6c242c99d5",
        strip_prefix = "ceres-solver-bd323ce698748bef0686eb27cb6cea4f88bb4f44",
        urls = ["https://github.com/ceres-solver/ceres-solver/archive/bd323ce698748bef0686eb27cb6cea4f88bb4f44.zip"],
    )

    http_archive(
        name = "com_google_tcmalloc",
        sha256 = "f11a004d7361ac6cd3e41fd573b08a92db28220934e8d4c82344ce0aeb20d1e4",
        strip_prefix = "tcmalloc-6c3e8bf43de02934525b3760571ca8781dca1869",
        url = "https://github.com/google/tcmalloc/archive/6c3e8bf43de02934525b3760571ca8781dca1869.zip",
    )

    http_archive(
        name = "tl-expected",
        build_file_content = """
cc_library(
  name = "tl-expected",
  srcs = ["include/tl/expected.hpp"],
  includes = ["include"],
  visibility = ["//visibility:public"],
)""",
        sha256 = "1db357f46dd2b24447156aaf970c4c40a793ef12a8a9c2ad9e096d9801368df6",
        strip_prefix = "expected-1.1.0",
        url = "https://github.com/TartanLlama/expected/archive/v1.1.0.tar.gz",
    )

    # Hedron's Compile Commands Extractor for Bazel
    # https://github.com/hedronvision/bazel-compile-commands-extractor
    http_archive(
        name = "hedron_compile_commands",

        # Replace the commit hash (daae6f40adfa5fdb7c89684cbe4d88b691c63b2d) in both places (below) with the latest (https://github.com/hedronvision/bazel-compile-commands-extractor/commits/main), rather than using the stale one here.
        # Even better, set up Renovate and let it do the work for you (see "Suggestion: Updates" in the README).
        sha256 = "43451a32bf271e7ba4635a07f7996d535501f066c0fe8feab04fb0c91dd5986e",
        strip_prefix = "bazel-compile-commands-extractor-daae6f40adfa5fdb7c89684cbe4d88b691c63b2d",
        url = "https://github.com/hedronvision/bazel-compile-commands-extractor/archive/daae6f40adfa5fdb7c89684cbe4d88b691c63b2d.tar.gz",
        # When you first run this tool, it'll recommend a sha256 hash to put here with a message like: "DEBUG: Rule 'hedron_compile_commands' indicated that a canonical reproducible form can be obtained by modifying arguments sha256 = ..."
    )

    # Source code of LZ4 (files under lib/) are under BSD 2-Clause.
    # The rest of the repository (build information, documentation, etc.) is under GPLv2.
    # We only care about the lib/ subfolder anyways, and strip out any other files.
    http_archive(
        name = "lz4",
        build_file = "@aos//debian:BUILD.lz4.bazel",
        sha256 = "0b0e3aa07c8c063ddf40b082bdf7e37a1562bda40a0ff5272957f3e987e0e54b",
        strip_prefix = "lz4-1.9.4",
        url = "https://github.com/lz4/lz4/archive/refs/tags/v1.9.4.tar.gz",
    )

    http_archive(
        name = "websocketpp",
        build_file = "@aos//third_party/websocketpp:websocketpp.BUILD",
        patch_args = ["-p1"],
        patches = ["@aos//third_party/websocketpp:websocketpp.patch"],
        sha256 = "6ce889d85ecdc2d8fa07408d6787e7352510750daa66b5ad44aacb47bea76755",
        strip_prefix = "websocketpp-0.8.2",
        url = "https://github.com/zaphoyd/websocketpp/archive/refs/tags/0.8.2.tar.gz",
    )

    http_archive(
        name = "com_github_storypku_bazel_iwyu",
        integrity = "sha256-R/rVwWn3SveoC8lAcicw6MOfdTqLLkubpaljT4qHjJg=",
        strip_prefix = "bazel_iwyu-bb102395e553215abd66603bcdeb6e93c66ca6d7",
        urls = [
            "https://github.com/storypku/bazel_iwyu/archive/bb102395e553215abd66603bcdeb6e93c66ca6d7.zip",
        ],
    )

    http_archive(
        name = "asio",
        build_file_content = """
cc_library(
    name = "asio",
    hdrs = glob(["include/asio/**/*.hpp", "include/asio/**/*.ipp", "include/asio.hpp"]),
    visibility = ["//visibility:public"],
    defines = ["ASIO_STANDALONE"],
    includes = ["include/"],
)""",
        sha256 = "8976812c24a118600f6fcf071a20606630a69afe4c0abee3b0dea528e682c585",
        strip_prefix = "asio-1.24.0",
        url = "https://downloads.sourceforge.net/project/asio/asio/1.24.0%2520%2528Stable%2529/asio-1.24.0.tar.bz2",
    )

    http_archive(
        name = "foxglove_ws_protocol",
        build_file = "@aos//third_party/foxglove/ws_protocol:foxglove_ws_protocol.BUILD",
        patch_args = ["-p1"],
        patches = ["@aos//third_party/foxglove/ws_protocol:foxglove_ws_protocol.patch"],
        sha256 = "eee484aefe4cb08dcef9ec52df5f904e017e15517865dcfa2462ff8070c9d906",
        strip_prefix = "ws-protocol-releases-typescript-ws-protocol-examples-v0.8.1",
        url = "https://github.com/foxglove/ws-protocol/archive/refs/tags/releases/typescript/ws-protocol-examples/v0.8.1.tar.gz",
    )

    http_archive(
        name = "com_github_foxglove_schemas",
        build_file = "@aos//:registry/modules/foxglove_schemas/0.16.2/overlay/BUILD.bazel",
        integrity = "sha256-O3/7+jBCOJu1HpPqGiJake4FVm3HlthRd5FQW4vHU2s=",
        strip_prefix = "foxglove-sdk-sdk-v0.16.2",
        url = "https://github.com/foxglove/foxglove-sdk/archive/refs/tags/sdk/v0.16.2.tar.gz",
    )

    # https://curl.haxx.se/download/curl-7.69.1.tar.gz
    http_archive(
        name = "com_github_curl_curl",
        build_file = "@aos//debian:curl.BUILD",
        sha256 = "01ae0c123dee45b01bbaef94c0bc00ed2aec89cb2ee0fd598e0d302a6b5e0a98",
        strip_prefix = "curl-7.69.1",
        url = "https://realtimeroboticsgroup.org/build-dependencies/curl-7.69.1.tar.gz",
    )

    http_archive(
        name = "com_github_nghttp2_nghttp2",
        build_file = "@aos//debian:BUILD.nghttp2.bazel",
        sha256 = "7da19947b33a07ddcf97b9791331bfee8a8545e6b394275a9971f43cae9d636b",
        strip_prefix = "nghttp2-1.58.0",
        url = "https://github.com/nghttp2/nghttp2/archive/refs/tags/v1.58.0.tar.gz",
    )

    http_archive(
        name = "com_github_grpc_grpc",
        patch_args = ["-p1"],
        patches = ["@aos//debian:grpc.patch"],
        sha256 = "7bf97c11cf3808d650a3a025bbf9c5f922c844a590826285067765dfd055d228",
        strip_prefix = "grpc-1.74.1",
        url = "https://github.com/grpc/grpc/archive/refs/tags/v1.74.1.tar.gz",
    )

    http_archive(
        name = "aspect_rules_ts",
        sha256 = "09af62a0d46918d815b5f48b5ed0f5349b62c15fc42fcc3fef5c246504ff8d99",
        strip_prefix = "rules_ts-3.6.3",
        url = "https://github.com/aspect-build/rules_ts/releases/download/v3.6.3/rules_ts-v3.6.3.tar.gz",
    )

    http_archive(
        name = "aspect_rules_js",
        sha256 = "b71565da7a811964e30cccb405544d551561e4b56c65f0c0aeabe85638920bd6",
        strip_prefix = "rules_js-2.4.2",
        url = "https://github.com/aspect-build/rules_js/releases/download/v2.4.2/rules_js-v2.4.2.tar.gz",
    )

    http_archive(
        name = "aspect_rules_rollup",
        sha256 = "0b8ac7d97cd660eb9a275600227e9c4268f5904cba962939d1a6ce9a0a059d2e",
        strip_prefix = "rules_rollup-2.0.1",
        url = "https://github.com/aspect-build/rules_rollup/releases/download/v2.0.1/rules_rollup-v2.0.1.tar.gz",
    )

    http_archive(
        name = "aspect_rules_esbuild",
        sha256 = "530adfeae30bbbd097e8af845a44a04b641b680c5703b3bf885cbd384ffec779",
        strip_prefix = "rules_esbuild-0.22.1",
        url = "https://github.com/aspect-build/rules_esbuild/releases/download/v0.22.1/rules_esbuild-v0.22.1.tar.gz",
    )

    http_archive(
        name = "aspect_rules_terser",
        sha256 = "c2013d66903fa42047b3bebeb4fc4a16ba380c310f772d8b28aaf8b5af6a1032",
        strip_prefix = "rules_terser-2.0.1",
        url = "https://github.com/aspect-build/rules_terser/releases/download/v2.0.1/rules_terser-v2.0.1.tar.gz",
        patches = [
            "@aos//third_party:rules_terser/0001-external-terser.patch",
        ],
        patch_args = ["-p1"],
    )

    http_archive(
        name = "aspect_rules_cypress",
        patch_args = ["-p1"],
        patches = [
            "@aos//third_party:rules_cypress/0001-fix-incorrect-linux-checksums.patch",
            "@aos//third_party:rules_cypress/0002-Add-support-for-cypress-13.6.6.patch",
        ],
        sha256 = "76947778d8e855eee3c15931e1fcdc1c2a25d56d6c0edd110b2227c05b794d08",
        strip_prefix = "rules_cypress-0.3.2",
        urls = [
            "https://github.com/aspect-build/rules_cypress/archive/refs/tags/v0.3.2.tar.gz",
        ],
    )

    # Copied from:
    # https://github.com/aspect-build/rules_cypress/blob/3db1b74818ac4ce1b9d489a6e0065b36c1076761/internal_deps.bzl#L47
    #
    # To update CHROME_REVISION, use the below script
    #
    # LASTCHANGE_URL="https://www.googleapis.com/download/storage/v1/b/chromium-browser-snapshots/o/Linux_x64%2FLAST_CHANGE?alt=media"
    # CHROME_REVISION=$(curl -s -S $LASTCHANGE_URL)
    # echo "latest CHROME_REVISION_LINUX is $CHROME_REVISION"
    CHROME_REVISION_LINUX = "1264932"

    http_archive(
        name = "chrome_linux",
        build_file_content = """filegroup(
    name = "all",
    srcs = glob(["**"]),
    visibility = ["//visibility:public"],
    )""",
        sha256 = "4de54f43b2fc4812b9fad4145e44df6ed3063969174a8883ea42ed4c1ee58301",
        strip_prefix = "chrome-linux",
        urls = [
            "https://www.googleapis.com/download/storage/v1/b/chromium-browser-snapshots/o/Linux_x64%2F" + CHROME_REVISION_LINUX + "%2Fchrome-linux.zip?alt=media",
        ],
    )

    http_archive(
        name = "chromedriver_linux",
        build_file_content = """
filegroup(
    name = "chromedriver",
    srcs = ["chromedriver-linux64/chromedriver"],
    visibility = ["//visibility:public"],
)""",
        sha256 = "527b81f8aaf94344af4103c1166ce5e65037e7ad071c773fe354c215d547ef73",
        urls = [
            "https://storage.googleapis.com/chrome-for-testing-public/124.0.6367.8/linux64/chromedriver-linux64.zip",
        ],
    )

    http_archive(
        name = "rules_rust_tinyjson",
        build_file = "@rules_rust//util/process_wrapper:BUILD.tinyjson.bazel",
        sha256 = "1a8304da9f9370f6a6f9020b7903b044aa9ce3470f300a1fba5bc77c78145a16",
        strip_prefix = "tinyjson-2.3.0",
        type = "tar.gz",
        url = "https://crates.io/api/v1/crates/tinyjson/2.3.0/download",
    )

    http_archive(
        name = "rules_m4",
        # Obtain the package checksum from the release page:
        # https://github.com/jmillikin/rules_m4/releases/tag/v0.2.4
        sha256 = "e2ada6a8d6963dc57fa56ef15be1894c37ddd85f6195b21eb290a835b1cef03a",
        urls = ["https://github.com/jmillikin/rules_m4/releases/download/v0.2.4/rules_m4-v0.2.4.tar.zst"],
    )

def frc_repositories():
    http_archive(
        name = "ctre_phoenix6_api_cpp_headers",
        build_file = "@aos//:registry/modules/ctre_phoenix6_api_cpp_headers/25.3.2/overlay/BUILD.bazel",
        sha256 = "76dc6139a275b19b537e7394c62d11a7d9ae2c65c0da8ac9b89cfc09b456ab1b",
        urls = [
            "https://maven.ctr-electronics.com/release/com/ctre/phoenix6/api-cpp/25.3.2/api-cpp-25.3.2-headers.zip",
        ],
    )

    http_archive(
        name = "ctre_phoenix6_api_cpp_athena",
        build_file = "@aos//:registry/modules/ctre_phoenix6_api_cpp_athena/25.3.2/overlay/BUILD.bazel",
        sha256 = "b8ee77b29891228a611ffe05fb94f5701e25d2970a3a54b502013faa83654b6a",
        urls = [
            "https://maven.ctr-electronics.com/release/com/ctre/phoenix6/api-cpp/25.3.2/api-cpp-25.3.2-linuxathena.zip",
        ],
    )

    http_archive(
        name = "ctre_phoenix6_tools_headers",
        build_file = "@aos//:registry/modules/ctre_phoenix6_tools_headers/25.3.2/overlay/BUILD.bazel",
        sha256 = "d26193e3e1be2d5bfea3de186364946a09ebd1ad5d40b32b8703293625c2b06d",
        urls = [
            "https://maven.ctr-electronics.com/release/com/ctre/phoenix6/tools/25.3.2/tools-25.3.2-headers.zip",
        ],
    )

    http_archive(
        name = "ctre_phoenix6_tools_athena",
        build_file = "@aos//:registry/modules/ctre_phoenix6_tools_athena/25.3.2/overlay/BUILD.bazel",
        sha256 = "92bdbdc87acb21f4b2b581f947fbf86cc268cfed761d847a47dd519d84c9ca58",
        urls = [
            "https://maven.ctr-electronics.com/release/com/ctre/phoenix6/tools/25.3.2/tools-25.3.2-linuxathena.zip",
        ],
    )

    http_archive(
        name = "ctre_phoenix_api_cpp_headers",
        build_file = "@aos//:registry/modules/ctre_phoenix_api_cpp_headers/5.35.0/overlay/BUILD.bazel",
        sha256 = "5abb072f9e5b6b3bcc86ddbfed4ecf4108d9ea85a6b91921633d45a88b4a0b0b",
        urls = [
            "https://maven.ctr-electronics.com/release/com/ctre/phoenix/api-cpp/5.35.0/api-cpp-5.35.0-headers.zip",
        ],
    )

    http_archive(
        name = "ctre_phoenix_api_cpp_athena",
        build_file = "@aos//:registry/modules/ctre_phoenix_api_cpp_athena/5.35.0/overlay/BUILD.bazel",
        sha256 = "b16d089f4f71804bbdb5245952de307b75410f2814392b6832f751177057c936",
        urls = [
            "https://maven.ctr-electronics.com/release/com/ctre/phoenix/api-cpp/5.35.0/api-cpp-5.35.0-linuxathena.zip",
        ],
    )

    http_archive(
        name = "ctre_phoenix_cci_headers",
        build_file = "@aos//:registry/modules/ctre_phoenix_cci_headers/5.35.0/overlay/BUILD.bazel",
        sha256 = "352fb8b0a73e18f0a00aa3c04880545c14a2bd09009031798a4f9a854ee71ff3",
        urls = [
            "https://maven.ctr-electronics.com/release/com/ctre/phoenix/cci/5.35.0/cci-5.35.0-headers.zip",
        ],
    )

    http_archive(
        name = "ctre_phoenix_cci_athena",
        build_file = "@aos//:registry/modules/ctre_phoenix_cci_athena/5.35.0/overlay/BUILD.bazel",
        sha256 = "cc8b1c4fb62368779cff97ef03ed6faf3612e7b327ba08a73849d789ea3a3b3c",
        urls = [
            "https://maven.ctr-electronics.com/release/com/ctre/phoenix/cci/5.35.0/cci-5.35.0-linuxathena.zip",
        ],
    )

    http_archive(
        name = "ctre_phoenix6_arm64",
        build_file = "@aos//:registry/modules/ctre_phoenix6_arm64/24.50.0/overlay/BUILD.bazel",
        sha256 = "0f1312f39eacc490fb253198c2d0e61e48ae00eff6a87cfd362358b1ad36a930",
        urls = [
            "https://realtimeroboticsgroup.org/build-dependencies/phoenix6_24.50.0-alpha-2_arm64-2024.10.26.tar.gz",
        ],
    )

    frc2026_field_map_welded_repo()

    intrinsic_calibration_test_images_repo()

    http_archive(
        name = "symengine",
        build_file = "@aos//debian:symengine.BUILD",
        sha256 = "1b5c3b0bc6a9f187635f93585649f24a18e9c7f2167cebcd885edeaaf211d956",
        strip_prefix = "symengine-0.12.0",
        url = "https://github.com/symengine/symengine/releases/download/v0.12.0/symengine-0.12.0.tar.gz",
    )

    calibrate_multi_cameras_data_repo()

    com_github_nvidia_cccl_repo()

    orin_image_apriltag_repo()

    orin_large_image_apriltag_repo()

    orin_capture_24_04_repo()

    orin_capture_24_04_side_repo()

    apriltag_test_bfbs_images_repo()

    http_archive(
        name = "halide_k8",
        build_file = "@aos//:registry/modules/halide_k8/14.0.0/overlay/BUILD.bazel",
        sha256 = "be3bdd067acb9ee0d37d0830821113cd69174bee46da466a836d8829fef7cf91",
        strip_prefix = "Halide-14.0.0-x86-64-linux/",
        url = "https://github.com/halide/Halide/releases/download/v14.0.0/Halide-14.0.0-x86-64-linux-6b9ed2afd1d6d0badf04986602c943e287d44e46.tar.gz",
    )

    http_archive(
        name = "halide_arm64",
        build_file = "@aos//:registry/modules/halide_arm64/14.0.0/overlay/BUILD.bazel",
        sha256 = "cdd42411bcbba682f73d7db0af69837c4857ee90f1727c6feb37fc9a98132385",
        strip_prefix = "Halide-14.0.0-arm-64-linux/",
        url = "https://github.com/halide/Halide/releases/download/v14.0.0/Halide-14.0.0-arm-64-linux-6b9ed2afd1d6d0badf04986602c943e287d44e46.tar.gz",
    )

    http_archive(
        name = "halide_darwin_aarch64",
        build_file = "@aos//:registry/modules/halide_darwin_aarch64/14.0.0/overlay/BUILD.bazel",
        sha256 = "164e5e6e238c7c35e54776d9911f2922d92078ef79a4a1f3c1c9cd5a251f3efa",
        strip_prefix = "Halide-14.0.0-arm-64-osx/",
        url = "https://github.com/halide/Halide/releases/download/v14.0.0/Halide-14.0.0-arm-64-osx-6b9ed2afd1d6d0badf04986602c943e287d44e46.tar.gz",
    )

    coral_image_thriftycam_2025_repo()

    drivetrain_replay_repo()

    superstructure_replay_repo()

def repositories(prefix = ""):
    aos_repositories(prefix = prefix)
    frc_repositories()
