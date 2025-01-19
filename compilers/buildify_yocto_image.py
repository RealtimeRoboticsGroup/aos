#!/usr/bin/python3

from __future__ import annotations
import sys
import unicodedata
import datetime
import os
import subprocess
from absl import app
from absl import flags

from rootfs_utils import scoped_tmpdir_tegraflash_rootfs, Filesystem, check_required_deps, check_buildifier, scoped_mount, generate_build_file

REQUIRED_DEPS = ["xfsprogs"]


def do_package(yocto_root, partition):
    tarball = datetime.date.today().strftime(
        f"{os.getcwd()}/%Y-%m-%d-scarthgap-arm64-nvidia-rootfs.tar")
    print(tarball, file=sys.stderr)

    subprocess.run([
        "sudo",
        "tar",
        "--sort=name",
        "--mtime=0",
        "--owner=0",
        "--group=0",
        "--numeric-owner",
        "--exclude=./usr/share/ca-certificates",
        "--exclude=./home",
        "--exclude=./root",
        "--exclude=./usr/src",
        "--exclude=./usr/lib/mesa-diverted",
        "--exclude=./usr/bin/X11",
        "--exclude=./usr/lib/systemd/system/system-systemd*cryptsetup.slice",
        "--exclude=./dev",
        "--exclude=./usr/local/cuda-12.6/bin/fatbinary",
        "--exclude=./usr/local/cuda-12.6/bin/ptxas",
        "--exclude=./usr/local/cuda-12.6/include/thrust",
        "--exclude=./usr/local/cuda-12.6/include/nv",
        "--exclude=./usr/local/cuda-12.6/include/cuda",
        "--exclude=./usr/local/cuda-12.6/include/cub",
        "--exclude=./usr/include/cub",
        "--exclude=./usr/include/nv",
        "--exclude=./usr/include/thrust",
        "--exclude=./usr/include/cuda",
        "--exclude=./usr/share",
        "-cf",
        tarball,
        ".",
    ],
                   cwd=partition,
                   check=True)

    # Pull ptxas and fatbinary from yocto.
    # Note, we need to use the downloaded but unmodified ones, otherwise yocto
    # modifies the binary and bakes paths in.
    #
    # To find the right binaries, run in build/tmp in yocto:
    #  (for line in $(find | grep ptxas$); do echo $line; file $line; ldd $line; done)
    cuda_bin = os.path.join(
        yocto_root,
        "build/tmp/work/x86_64-linux/cuda-nvcc-headers-native/12.6.68-1/cuda-nvcc-headers-12.6.68-1/usr/local/cuda-12.6/bin"
    )

    subprocess.run([
        "sudo",
        "tar",
        "--sort=name",
        "--mtime=0",
        "--owner=0",
        "--group=0",
        "--numeric-owner",
        f'--transform=s|{cuda_bin[1:]}/ptxas|usr/local/cuda-12.6/bin/ptxas|',
        f'--transform=s|{cuda_bin[1:]}/fatbinary|usr/local/cuda-12.6/bin/aarch64-unknown-linux-gnu-fatbinary|',
        "--append",
        "-f",
        tarball,
        os.path.join(cuda_bin, "ptxas"),
        os.path.join(cuda_bin, "fatbinary"),
    ],
                   check=True)

    subprocess.run(["sha256sum", tarball], check=True)


def main(argv):
    check_required_deps(REQUIRED_DEPS)

    if len(argv) != 2:
        print("Usage:", file=sys.stderr)
        print(" buildify_yocto_image /path/to/yocto", file=sys.stderr)
        return 1

    if argv[1][0] != '/':
        print("Path must be absolute", file=sys.stderr)
        return 1

    full_image = os.path.join(
        argv[1],
        "build/demo-image-base-jetson-orin-nano-som.rootfs.tegraflash.tar.zst")

    if not check_buildifier():
        print(
            "ERROR: Need to have buildifier in the path.  Please resolve this.",
            file=sys.stderr)
        exit()

    if not os.path.exists(full_image):
        print("ERROR: Point to a valid image", file=sys.stderr)
        exit()

    with scoped_tmpdir_tegraflash_rootfs(full_image) as rootfs:
        with scoped_mount(rootfs) as partition:
            subprocess.run([
                "sudo", "cp", "/usr/bin/qemu-aarch64-static",
                f"{partition}/usr/bin/"
            ],
                           check=True)

            subprocess.run(["sudo", "chmod", "a+X", "-R", f"{partition}/"],
                           check=True)

            filesystem = Filesystem(
                partition,
                [
                    "sudo", "chroot", "--userspec=0:0", partition,
                    "qemu-aarch64-static", "/bin/bash"
                ],
            )

            packages_to_eval = [
                filesystem.packages['libglib-2.0-dev'],
                filesystem.packages['opencv-dev'],
                filesystem.packages['opencv'],
                filesystem.packages['libz-dev'],
                filesystem.packages['libc6-dev'],
                filesystem.packages['xz-dev'],
                filesystem.packages['libstdc++-dev'],
                filesystem.packages['libffi-dev'],
                filesystem.packages['util-linux-dev'],
                filesystem.packages['libnpp-dev'],
                filesystem.packages['cuda-cudart-dev'],
                filesystem.packages['gstreamer1.0-dev'],
                filesystem.packages['orc-dev'],
                filesystem.packages['libgstrtp-1.0-0'],
                filesystem.packages['libpcre2-dev'],
                filesystem.packages['gstreamer1.0-plugins-bad-dev'],
            ]

            with open("orin_debian_rootfs.BUILD", "w") as file:
                file.write(
                    generate_build_file(filesystem, packages_to_eval,
                                        "orin_debian_rootfs.BUILD.template"))

            subprocess.run(['buildifier', "orin_debian_rootfs.BUILD"])

            do_package(argv[1], partition)


if __name__ == '__main__':
    app.run(main)
