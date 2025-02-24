# Working with sysroots

## Theory of operation

It is impossible to build everything from source with Bazel.
It even borders on undesirable.
Things like yocto and other linux distributions do a good job configuring, compiling, and packaging up packages.

Unfortunately, Bazel doesn't really like linking against sysroots.
It wants to know what headers go with each library, and all the other options.
It is a lot of work to figure this all out.

Instead, we can write a python script to compute all of this and automatically generate BUILD files to compile against everything.
It uses `objdump`, `pkg-config`, `ldconfig`, and `dpkg` to compute the dependency graph automatically and generate both `cc_library` targets, and `filegroup` targets for package sources.  It generates `cc_library` targets which represent each of the options from `pkg-config` to make it easy.

There are enough weird packages that it is easier to tell the script the packages to start from than to fix all the weird ones.

# Usage for Debian Bookworm

First, build the root filesystem image in `//frc/amd64` (or download the one in the WORKSPACE file).

Then, run:

```
./buildify_debian_x86_image.py ../frc/amd64/2025-02-22-bookworm-amd64-nvidia-rootfs.tar.zst
```

It will update `amd64_debian_rootfs.BUILD`, which needs to be checked in.

# Usage for Ubuntu Jammy

First, build the root filesystem image in `//frc/amd64` (or download the one in the WORKSPACE file).

Then, run:

```
./buildify_debian_x86_image.py ../frc/amd64/2025-02-23-jammy-amd64-nvidia-rootfs.tar
```

It will update `amd64_ubuntu_rootfs.BUILD`, which needs to be checked in.
