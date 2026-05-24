package(default_visibility = ["//visibility:public"])

platform(
    name = "linux_x86",
    constraint_values = [
        "@platforms//os:linux",
        "@platforms//cpu:x86_64",
        "//tools/platforms/go:has_support",
        "//tools/platforms/rust:has_support",
        "//tools/platforms/nodejs:has_support",
        "//tools/platforms/python:has_support",
        "//tools/platforms/gpu:nvidia",
    ],
    exec_properties = {
        "OSFamily": "Linux",
        # Container built from: tools/platforms/debian-bookworm/Dockerfile
        # docker build -t austinschuh/spacecookies-engflow:latest .
        # docker push austinschuh/spacecookies-engflow:latest
        # docker inspect docker.io/austinschuh/spacecookies-engflow:latest | grep RepoDigests -C 5
        "container-image": "docker://austinschuh/spacecookies-engflow@sha256:fffc72bacc97ac3dd4b25d5e6557fef2ef2b2decee2bd58a9c50934186a8d219",
        # Let asan work.
        "dockerAddCapabilities": "SYS_PTRACE",
        "dockerReuse": "True",
    },
)

platform(
    name = "linux_x86_no_gpu",
    constraint_values = [
        "@platforms//os:linux",
        "@platforms//cpu:x86_64",
        "//tools/platforms/go:has_support",
        "//tools/platforms/rust:has_support",
        "//tools/platforms/nodejs:has_support",
        "//tools/platforms/python:has_support",
        "//tools/platforms/gpu:none",
    ],
    exec_properties = {
        "OSFamily": "Linux",
        # Container built from: tools/platforms/debian-bookworm/Dockerfile
        # docker build -t austinschuh/spacecookies-engflow:latest .
        # docker push austinschuh/spacecookies-engflow:latest
        # docker inspect docker.io/austinschuh/spacecookies-engflow:latest | grep RepoDigests -C 5
        "container-image": "docker://austinschuh/spacecookies-engflow@sha256:fffc72bacc97ac3dd4b25d5e6557fef2ef2b2decee2bd58a9c50934186a8d219",
        # Let asan work.
        "dockerAddCapabilities": "SYS_PTRACE",
        "dockerReuse": "True",
    },
)

platform(
    name = "linux_arm64",
    constraint_values = [
        "@platforms//os:linux",
        "@platforms//cpu:arm64",
        "//tools/platforms/hardware:raspberry_pi",
        "//tools/platforms/go:lacks_support",
        "//tools/platforms/rust:has_support",
        "//tools/platforms/nodejs:lacks_support",
        "//tools/platforms/python:has_support",
        "//tools/platforms/gpu:nvidia",
    ],
)

platform(
    name = "linux_roborio",
    constraint_values = [
        "@platforms//os:linux",
        "@platforms//cpu:armv7",
        "//tools/platforms/hardware:roborio",
        "//tools/platforms/go:lacks_support",
        # TODO(Brian): This almost works, but cxx assumes llvm-ld's linking
        # behavior and doesn't have an easy way to support GNU ld. See
        # https://github.com/dtolnay/cxx/pull/1069 for a bit more explanation.
        # Bazel needs to group multiple things into a single cc_library to
        # handle that, need to figure out how to do that here or switch linkers.
        "//tools/platforms/rust:lacks_support",
        "//tools/platforms/nodejs:lacks_support",
        "//tools/platforms/python:lacks_support",
        "//tools/platforms/gpu:none",
        "@rules_bzlmodrio_toolchains//constraints/is_roborio:true",
    ],
)

platform(
    name = "cortex_m4f",
    constraint_values = [
        "@platforms//os:none",
        "//tools/platforms/hardware:cortex_m4f",
        "//tools/platforms/go:lacks_support",
        "//tools/platforms/rust:lacks_support",
        "//tools/platforms/nodejs:lacks_support",
        "//tools/platforms/python:lacks_support",
        "//tools/platforms/gpu:none",
    ],
)

platform(
    name = "rp2040",
    constraint_values = [
        "@platforms//os:none",
        "//tools/platforms/hardware:cortex_m0plus",
        "//tools/platforms/go:lacks_support",
        "//tools/platforms/rust:lacks_support",
        "//tools/platforms/nodejs:lacks_support",
        "//tools/platforms/python:lacks_support",
        "//tools/platforms/gpu:none",
    ],
)

platform(
    name = "cortex-m4f-imu",
    constraint_values = [
        "@platforms//os:none",
        "//tools/platforms/hardware:cortex-m4f-imu",
        "//tools/platforms/go:lacks_support",
        "//tools/platforms/rust:lacks_support",
        "//tools/platforms/nodejs:lacks_support",
        "//tools/platforms/python:lacks_support",
        "//tools/platforms/gpu:none",
    ],
)

# TODO(phil): Create something for "cortex-m4f-k22" builds.

platform(
    name = "darwin_arm64",
    constraint_values = [
        "@platforms//os:macos",
        "@platforms//cpu:arm64",
        "//tools/platforms/go:has_support",
        "//tools/platforms/rust:has_support",
        "//tools/platforms/nodejs:has_support",
        "//tools/platforms/python:has_support",
        "//tools/platforms/gpu:none",
    ],
    flags = [
        # We need at least 14.4 for os_sync_wait_on_address (used by aos_sync.cc).
        "--macos_minimum_os=14.4",
    ],
)
