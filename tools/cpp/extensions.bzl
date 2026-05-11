load("//:repositories_internal.bzl", "arm_frc_linux_gnueabi_repo_repo", "gcc_arm_none_eabi_repo")

def _dev_toolchains_extension_impl(_ctx):
    arm_frc_linux_gnueabi_repo_repo()

    gcc_arm_none_eabi_repo()

dev_toolchains_extension = module_extension(
    implementation = _dev_toolchains_extension_impl,
)
