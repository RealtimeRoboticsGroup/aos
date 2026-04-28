load("@toolchains_llvm//toolchain:rules.bzl", "llvm", "toolchain")
load("//:repositories_internal.bzl", "arm_frc_linux_gnueabi_repo_repo", "gcc_arm_none_eabi_repo")

def _llvm_toolchain_extension_impl(_ctx):
    llvm_version = "21.1.1"

    # We use custom-built `.tar.zst` clang distributions (matching
    # upstream LLVM 21.1.1 sources) hosted on our own mirror.
    # `extra_llvm_distributions` registers their SHAs and
    # `alternative_llvm_sources` provides the URL pattern, both of which
    # are public knobs on `toolchains_llvm`'s `llvm()` repo rule, so no
    # patching of upstream's distribution table is required.
    llvm_alternative_sources = [
        "https://mirror.spacecookies.dev/github.com/llvm/llvm-project/releases/download/llvmorg-{llvm_version}/{basename}",
    ]
    llvm_x86_64_basename = "clang+llvm-%s-x86_64-linux-gnu-ubuntu-22.04.tar.zst" % llvm_version
    llvm_aarch64_basename = "clang+llvm-%s-aarch64-linux-gnu.tar.zst" % llvm_version
    llvm_extra_distributions = {
        llvm_x86_64_basename: "75dde978fcfe30486680e9d2fdbad7e92d9b44b48dea8193023399bc7485f885",
        llvm_aarch64_basename: "f9b33b7ed6cd693160922873a8ae7ec1aadf6ad1efc8e2bee13625b4dc787ce6",
    }

    llvm(
        name = "llvm_k8",
        alternative_llvm_sources = llvm_alternative_sources,
        distribution = llvm_x86_64_basename,
        extra_llvm_distributions = llvm_extra_distributions,
        llvm_versions = {"": llvm_version},
    )

    llvm(
        name = "llvm_aarch64",
        alternative_llvm_sources = llvm_alternative_sources,
        distribution = llvm_aarch64_basename,
        extra_llvm_distributions = llvm_extra_distributions,
        llvm_versions = {"": llvm_version},
    )

    llvm_conly_flags = ["-std=gnu99"]
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

    toolchain(
        name = "llvm_toolchain",
        conly_flags = {
            "linux-aarch64": llvm_conly_flags,
            "linux-x86_64": llvm_conly_flags,
        },
        cxx_standard = {
            "linux-aarch64": llvm_cxx_standard,
            "linux-x86_64": llvm_cxx_standard,
        },
        extra_compile_flags = {
            "linux-aarch64": llvm_extra_compile_flags_aarch64,
            "linux-x86_64": llvm_extra_compile_flags,
        },
        llvm_versions = {"": llvm_version},
        # The arm64 sysroot is a Yocto rootfs whose libstdc++ uses the
        # `aarch64-oe4t-linux` multiarch tuple instead of the Debian
        # `aarch64-linux-gnu` default baked into toolchains_llvm, and lays
        # out its libstdc++ headers / gcc runtime under
        # `/usr/include/c++/<ver>/<multiarch>` and `/usr/lib/<multiarch>/<ver>`
        # respectively (Yocto), not the Debian
        # `/usr/include/<multiarch>/c++/<ver>` and `/usr/lib/gcc/<multiarch>/<ver>`.
        multiarch = {
            "linux-aarch64": "aarch64-oe4t-linux",
        },
        cxx_include_layout = {
            "linux-aarch64": "yocto",
        },
        stdlib = {
            "linux-aarch64": "dynamic-stdc++-14.3.0",
            "linux-x86_64": "dynamic-stdc++-12",
        },
        sysroot = {
            "linux-aarch64": "@@arm64_debian_sysroot+//:sysroot_files",
            "linux-x86_64": "@@amd64_debian_sysroot+//:sysroot_files",
        },
        target_toolchain_roots = {
            "linux-aarch64": "@@+llvm_toolchain_extension+llvm_aarch64//",
            "linux-x86_64": "@@+llvm_toolchain_extension+llvm_k8//",
        },
        toolchain_roots = {
            "linux-aarch64": "@@+llvm_toolchain_extension+llvm_aarch64//",
            "linux-x86_64": "@@+llvm_toolchain_extension+llvm_k8//",
        },
    )

llvm_toolchain_extension = module_extension(
    implementation = _llvm_toolchain_extension_impl,
)

def _dev_toolchains_extension_impl(_ctx):
    arm_frc_linux_gnueabi_repo_repo()

    gcc_arm_none_eabi_repo()

dev_toolchains_extension = module_extension(
    implementation = _dev_toolchains_extension_impl,
)
