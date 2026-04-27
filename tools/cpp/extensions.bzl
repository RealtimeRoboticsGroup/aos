load("@toolchains_llvm//toolchain:rules.bzl", "llvm", "toolchain")
load("//:repositories_internal.bzl", "arm_frc_linux_gnueabi_repo_repo", "gcc_arm_none_eabi_repo")

def _llvm_toolchain_extension_impl(_ctx):
    llvm_version = "21.1.1"

    llvm(
        name = "llvm_k8",
        distribution = "clang+llvm-%s-x86_64-linux-gnu-ubuntu-22.04.tar.zst" % llvm_version,
        llvm_versions = {"": llvm_version},
    )

    llvm(
        name = "llvm_aarch64",
        distribution = "clang+llvm-%s-aarch64-linux-gnu.tar.zst" % llvm_version,
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
