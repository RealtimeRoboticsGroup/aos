load("@rules_python//python:defs.bzl", "py_runtime", "py_runtime_pair")
load("@rules_shell//shell:sh_binary.bzl", "sh_binary")
load("@rules_uv//uv:pip.bzl", "pip_compile")
load("//tools/python:defs.bzl", "expand_template", "py_binary")

exports_files(["runtime_binary.sh.tpl"])

# Invoke this via "bazel run //tools/python:requirements.update".
pip_compile(
    name = "requirements.update",
    args = [
        # Generate hashes for reproducibility.
        # TODO(philsc): Uncomment once we migrate to blzmod and a uv lock file.
        #"--generate-hashes",
        # Work around dependencies only being pulled from the same mirror.
        # https://github.com/astral-sh/uv/pull/2083
        "--index-strategy=unsafe-any-match",
        # For compatibility with rules_python and pip, we keep the extras in the
        # requirements.
        "--no-strip-extras",
        # Grab the hashes from both our own index and the upstream index. This
        # lets the `pip wheel` command generate wheels properly. We restrict the
        # downloads in WORKSPACE sufficiently for CI purposes.
        "--default-index=https://realtimeroboticsgroup.org/build-dependencies/wheelhouse/simple",
        "--index=https://pypi.org/simple",
    ],
    requirements_in = "requirements.txt",
    requirements_txt = "requirements.lock.txt",
    tags = [
        # The test pings pypi.org to make sure that the lock file matches the
        # requirements file.
        "requires-network",
        # The test causes some packages' setup.py to be executed which can
        # execute arbitrary code. Sometimes they look for compilers etc. That's
        # not good for our hermeticity assumptions. Disable the test lock file
        # test for now.
        "manual",
    ],
    target_compatible_with = [
        # TODO(philsc): Make Python work for other platforms too.
        "@platforms//cpu:x86_64",
    ],
)

# Invoke this via "bazel run //tools/python:aos_requirements.update".
pip_compile(
    name = "aos_requirements.update",
    args = [
        "--index-strategy=unsafe-any-match",
        "--no-strip-extras",
        "--default-index=https://realtimeroboticsgroup.org/build-dependencies/wheelhouse/simple",
        "--index=https://pypi.org/simple",
        "--constraint=tools/python/requirements.lock.txt",
    ],
    data = ["requirements.lock.txt"],
    requirements_in = "aos_requirements.txt",
    requirements_txt = "aos_requirements.lock.txt",
    tags = [
        "manual",
        "requires-network",
    ],
    target_compatible_with = [
        "@platforms//cpu:x86_64",
    ],
)

# This binary is intended to run the `requirements.update` target in a Docker
# container. This is primarily intended for reproducibility. See README.md.
sh_binary(
    name = "update_helper",
    srcs = ["update_helper.sh"],
    data = [
        "update_helper_files/Dockerfile",
    ],
    deps = [
        "@bazel_tools//tools/bash/runfiles",
    ],
)

expand_template(
    name = "gen_runtime_binary_bzlmod_3_10",
    out = "runtime_binary_bzlmod_3_10.sh",
    substitutions = {
        "@NCCL_DIR_NAME@": "rules_python++pip+aos_pip_deps_310_nvidia_nccl_cu12",
        "@PYTHON_REPO_DIR@": "rules_python++python+python_3_10_x86_64-unknown-linux-gnu",
        "@PYTHON_VER@": "3_10",
        "@SHORT_VER@": "310",
        "@SYSROOT_DIR_NAME@": "amd64_debian_sysroot+",
    },
    template = "runtime_binary.sh.tpl",
    visibility = ["//visibility:public"],
)

expand_template(
    name = "gen_runtime_binary_workspace_3_10",
    out = "runtime_binary_workspace_3_10.sh",
    substitutions = {
        "@NCCL_DIR_NAME@": "pip_deps_nvidia_nccl_cu12",
        "@PYTHON_REPO_DIR@": "python_3_10_x86_64-unknown-linux-gnu",
        "@PYTHON_VER@": "3_10",
        "@SHORT_VER@": "310",
        "@SYSROOT_DIR_NAME@": "amd64_debian_sysroot",
    },
    template = "runtime_binary.sh.tpl",
    visibility = ["//visibility:public"],
)

expand_template(
    name = "gen_runtime_binary_bzlmod_3_13",
    out = "runtime_binary_bzlmod_3_13.sh",
    substitutions = {
        "@NCCL_DIR_NAME@": "rules_python++pip+aos_pip_deps_313_nvidia_nccl_cu12",
        "@PYTHON_REPO_DIR@": "rules_python++python+python_3_13_x86_64-unknown-linux-gnu",
        "@PYTHON_VER@": "3_13",
        "@SHORT_VER@": "313",
        "@SYSROOT_DIR_NAME@": "amd64_debian_sysroot+",
    },
    template = "runtime_binary.sh.tpl",
    visibility = ["//visibility:public"],
)

expand_template(
    name = "gen_runtime_binary_workspace_3_13",
    out = "runtime_binary_workspace_3_13.sh",
    substitutions = {
        "@NCCL_DIR_NAME@": "pip_deps_nvidia_nccl_cu12",
        "@PYTHON_REPO_DIR@": "python_3_13_x86_64-unknown-linux-gnu",
        "@PYTHON_VER@": "3_13",
        "@SHORT_VER@": "313",
        "@SYSROOT_DIR_NAME@": "amd64_debian_sysroot",
    },
    template = "runtime_binary.sh.tpl",
    visibility = ["//visibility:public"],
)

py_runtime(
    name = "python3_10_bzlmod_runtime",
    files = [
        "runtime_binary_bzlmod_3_10.sh",
        "@python_3_10_x86_64-unknown-linux-gnu//:files",
    ],
    interpreter = "runtime_binary_bzlmod_3_10.sh",
    interpreter_version_info = {
        "major": "3",
        "micro": "18",
        "minor": "10",
    },
    python_version = "PY3",
)

py_runtime(
    name = "python3_10_workspace_runtime",
    files = [
        "runtime_binary_workspace_3_10.sh",
        "@python_3_10_x86_64-unknown-linux-gnu//:files",
    ],
    interpreter = "runtime_binary_workspace_3_10.sh",
    interpreter_version_info = {
        "major": "3",
        "micro": "18",
        "minor": "10",
    },
    python_version = "PY3",
)

py_runtime(
    name = "python3_13_bzlmod_runtime",
    files = [
        "runtime_binary_bzlmod_3_13.sh",
        "@python_3_13_x86_64-unknown-linux-gnu//:files",
    ],
    interpreter = "runtime_binary_bzlmod_3_13.sh",
    interpreter_version_info = {
        "major": "3",
        "micro": "0",
        "minor": "13",
    },
    python_version = "PY3",
)

py_runtime(
    name = "python3_13_workspace_runtime",
    files = [
        "runtime_binary_workspace_3_13.sh",
        "@python_3_13_x86_64-unknown-linux-gnu//:files",
    ],
    interpreter = "runtime_binary_workspace_3_13.sh",
    interpreter_version_info = {
        "major": "3",
        "micro": "0",
        "minor": "13",
    },
    python_version = "PY3",
)

py_runtime_pair(
    name = "py_bzlmod_runtime",
    py2_runtime = None,
    py3_runtime = select({
        "@rules_python//python/config_settings:is_python_3.13": ":python3_13_bzlmod_runtime",
        "//conditions:default": ":python3_10_bzlmod_runtime",
    }),
)

py_runtime_pair(
    name = "py_workspace_runtime",
    py2_runtime = None,
    py3_runtime = select({
        "@rules_python//python/config_settings:is_python_3.13": ":python3_13_workspace_runtime",
        "//conditions:default": ":python3_10_workspace_runtime",
    }),
)

toolchain(
    name = "python_bzlmod_toolchain",
    exec_compatible_with = [
        "@platforms//cpu:x86_64",
        "@platforms//os:linux",
    ],
    target_compatible_with = [
        "@platforms//cpu:x86_64",
        "@platforms//os:linux",
    ],
    toolchain = ":py_bzlmod_runtime",
    toolchain_type = "@rules_python//python:toolchain_type",
)

toolchain(
    name = "python_workspace_toolchain",
    exec_compatible_with = [
        "@platforms//cpu:x86_64",
        "@platforms//os:linux",
    ],
    target_compatible_with = [
        "@platforms//cpu:x86_64",
        "@platforms//os:linux",
    ],
    toolchain = ":py_workspace_runtime",
    toolchain_type = "@rules_python//python:toolchain_type",
)

alias(
    name = "python_toolchain",
    actual = ":python_bzlmod_toolchain",
)

py_binary(
    name = "mirror_pip_packages",
    srcs = ["mirror_pip_packages.py"],
    deps = [
        "@pip_deps//pkginfo",
        "@pip_deps//requests",
    ],
)

py_runtime(
    name = "noop_runtime",
    interpreter = "noop_error_exit.sh",
    python_version = "PY3",
)

py_runtime_pair(
    name = "noop_py_runtime_pair",
    py2_runtime = None,
    py3_runtime = ":noop_runtime",
)

toolchain(
    name = "noop_python_toolchain",
    exec_compatible_with = [
        "@platforms//os:linux",
    ],
    target_compatible_with = [
        "//tools/platforms/python:lacks_support",
    ],
    toolchain = ":noop_py_runtime_pair",
    toolchain_type = "@rules_python//python:toolchain_type",
)
