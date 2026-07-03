#!/usr/bin/env python3

import argparse
import platform
import shutil
import subprocess
import sys
from pathlib import Path

LLVM_RELEASE_ARCH = {
    ("Linux", "x86_64"): "x86_64-linux-gnu-ubuntu-22.04",
    ("Linux", "aarch64"): "aarch64-linux-gnu",
    ("Darwin", "arm64"): "aarch64-apple-darwin",
    ("Darwin", "x86_64"): "x86_64-apple-darwin",
}

LIBCXX_SANITIZERS = {
    "msan": "MemoryWithOrigins",
}


def run(*args, cwd=None):
    subprocess.check_call([str(a) for a in args],
                          cwd=str(cwd) if cwd else None)


def require_programs(programs):
    missing = [p for p in programs if shutil.which(p) is None]
    if missing:
        print("Missing required programs: {}".format(", ".join(
            sorted(missing))),
              file=sys.stderr)
        sys.exit(1)


def ensure_dir(path: Path):
    path.mkdir(parents=True, exist_ok=True)


def clone_llvm_source(source_dir: Path, llvm_version: str):
    if source_dir.exists():
        shutil.rmtree(source_dir)
    run(
        "git",
        "clone",
        "--depth=1",
        "https://github.com/llvm/llvm-project",
        "--branch",
        "llvmorg-{}".format(llvm_version),
        source_dir,
    )
    patches_dir = Path(__file__).resolve().parents[2] / "third_party/llvm"
    if patches_dir.exists():
        for patch in sorted(patches_dir.glob("*.patch")):
            print("Applying patch: {}".format(patch.name), file=sys.stderr)
            run("git", "apply", patch, cwd=source_dir)


def download_extract_clang_distribution(clang_url: str, clang_dir: Path):
    if clang_dir.exists():
        shutil.rmtree(clang_dir)
    ensure_dir(clang_dir)

    curl_proc = subprocess.Popen(["curl", "-L", "-s", clang_url],
                                 stdout=subprocess.PIPE)
    tar_proc = subprocess.Popen(
        [
            "tar",
            "xJ",
            "-C",
            str(clang_dir),
            "--strip-components=1",
        ],
        stdin=curl_proc.stdout,
    )
    curl_proc.stdout.close()
    tar_rc = tar_proc.wait()
    curl_rc = curl_proc.wait()
    if curl_rc != 0 or tar_rc != 0:
        raise subprocess.CalledProcessError(curl_rc or tar_rc, "curl|tar")


def build_clang_release(
    tempdir: Path,
    source_dir: Path,
    clang_topdir: Path,
    clang_dir: Path,
    clang_basename: str,
    force_rebuild: bool,
):
    build_dir = tempdir / "clang-build"
    build_success_file = tempdir / "clang-build.success"
    install_success_file = tempdir / "clang-install.success"
    clang_basename_file = tempdir / "clang-basename.txt"

    previous_basename = None
    if clang_basename_file.exists():
        previous_basename = clang_basename_file.read_text().strip()

    if force_rebuild:
        build_success_file.unlink(missing_ok=True)
        install_success_file.unlink(missing_ok=True)

    if previous_basename and previous_basename != clang_basename:
        build_success_file.unlink(missing_ok=True)
        install_success_file.unlink(missing_ok=True)

    if install_success_file.exists() and not clang_dir.exists():
        build_success_file.unlink(missing_ok=True)
        install_success_file.unlink(missing_ok=True)

    if not build_success_file.exists() or not install_success_file.exists():
        for directory, success_file in ((build_dir, build_success_file),
                                        (clang_topdir, install_success_file)):
            if not success_file.exists():
                if directory.exists():
                    shutil.rmtree(directory)
                ensure_dir(directory)

    if not build_success_file.exists():
        run(
            "cmake",
            "-GNinja",
            source_dir / "llvm",
            "-DCMAKE_BUILD_TYPE=Release",
            "-DLLVM_ENABLE_PROJECTS=clang;clang-tools-extra;lld;compiler-rt;",
            "-DLLVM_ENABLE_RUNTIMES=libcxx;libcxxabi;libunwind",
            "-DCMAKE_C_COMPILER={}".format(shutil.which("clang") or "clang"),
            "-DCMAKE_CXX_COMPILER={}".format(
                shutil.which("clang++") or "clang++"),
            "-DLLVM_ENABLE_TERMINFO=OFF",
            "-DLLVM_ENABLE_ZLIB=FORCE_ON",
            "-DLLVM_ENABLE_ZSTD=FORCE_ON",
            "-DCMAKE_INSTALL_PREFIX={}".format(clang_dir),
            cwd=build_dir,
        )
        run("ninja", cwd=build_dir)
        build_success_file.touch()
        print("Clang build successful", file=sys.stderr)

    if not install_success_file.exists():
        run("ninja", "install", cwd=build_dir)
        install_success_file.touch()
        clang_basename_file.write_text(clang_basename + "\n")
        print("Clang install successful", file=sys.stderr)

    tar_path = tempdir / (clang_basename + ".tar.zst")
    if force_rebuild or not tar_path.exists():
        run(
            "tar",
            "cf",
            tar_path,
            "--use-compress-program=zstd --ultra -21 -T0 -",
            ".",
            cwd=clang_topdir,
        )
        print("Built {}".format(tar_path), file=sys.stderr)
    else:
        print("Already exists {}".format(tar_path), file=sys.stderr)


def build_libcxx_variant(
    variant: str,
    source_dir: Path,
    tempdir: Path,
    clang_dir: Path,
    llvm_version: str,
    release_arch: str,
):
    if variant == "msan" and platform.system() == "Darwin":
        print("Skipping msan on Darwin (unsupported)", file=sys.stderr)
        return

    if variant not in LIBCXX_SANITIZERS:
        raise ValueError("unsupported libcxx variant '{}'".format(variant))

    build_dir = tempdir / "libcxx-{}-build".format(variant)
    if build_dir.exists():
        shutil.rmtree(build_dir)
    ensure_dir(build_dir)

    install_prefix = build_dir / "install_prefix"
    sanitizer = LIBCXX_SANITIZERS[variant]

    run(
        "cmake",
        "-GNinja",
        source_dir / "runtimes",
        "-DCMAKE_BUILD_TYPE=Release",
        "-DLLVM_ENABLE_RUNTIMES=libcxx;libcxxabi;libunwind",
        "-DCMAKE_C_COMPILER={}".format(clang_dir / "bin/clang"),
        "-DCMAKE_CXX_COMPILER={}".format(clang_dir / "bin/clang++"),
        "-DLLVM_USE_SANITIZER={}".format(sanitizer),
        "-DCMAKE_INSTALL_PREFIX={}".format(install_prefix),
        "-DLLVM_ENABLE_PIC=ON",
        cwd=build_dir,
    )
    run("ninja", cwd=build_dir)
    run("ninja", "install", cwd=build_dir)

    machine = platform.machine()
    system = platform.system()
    if system == "Linux":
        target_subdir = "{}-unknown-linux-gnu".format(machine) if machine in (
            "x86_64", "aarch64") else ""
    elif system == "Darwin":
        target_subdir = "{}-apple-darwin".format(machine)
    else:
        target_subdir = ""

    for ext in ("a", "so", "so.1", "so.1.0", "dylib"):
        src = clang_dir / "lib"
        if target_subdir:
            src = src / target_subdir
        src = src / ("libunwind." + ext)
        if src.exists():
            shutil.copy(src, install_prefix / "lib")

    tar_path = tempdir / "libcxx-{}-{}-{}.tar.zst".format(
        variant, llvm_version, release_arch)
    run(
        "tar",
        "cf",
        tar_path,
        "-C",
        install_prefix,
        "--use-compress-program=zstd --ultra -21 -T0 -",
        "lib",
        "include",
    )
    print("Built {}".format(tar_path), file=sys.stderr)


def main():
    parser = argparse.ArgumentParser(
        description=
        "Build LLVM release tarballs and/or libcxx variants from llvm-project source. "
        "See tools/helpers/README.md for detailed usage instructions and examples."
    )
    parser.add_argument("--llvm-version",
                        required=True,
                        help="LLVM version to clone (e.g. 21.1.1).")
    parser.add_argument(
        "--release-arch",
        default=None,
        help=
        "LLVM release architecture suffix (default: inferred from host, e.g. Linux-X64, Linux-ARM64).",
    )
    parser.add_argument(
        "--libcxx",
        action="extend",
        nargs="+",
        default=[],
        help="Libcxx variants to build (supported: {}).".format(", ".join(
            sorted(LIBCXX_SANITIZERS.keys()))),
    )
    parser.add_argument(
        "--tempdir",
        type=Path,
        default=Path("/tmp/llvm_build_temp"),
        help="Top-level build directory.",
    )
    parser.add_argument(
        "--clang",
        action="store_true",
        help="Build a clang release-style tarball from source.")
    parser.add_argument(
        "--clang-force-rebuild",
        action="store_true",
        help=
        "Force rebuilding/repackaging clang even if success markers exist.",
    )
    parser.add_argument("--force-redownload",
                        action="store_true",
                        help="Force a fresh llvm-project clone.")

    args = parser.parse_args()

    machine = platform.machine()
    system = platform.system()
    release_arch = args.release_arch or LLVM_RELEASE_ARCH.get(
        (system, machine))
    if not release_arch:
        print(
            "Unable to infer release arch from '{}-{}'. Pass --release-arch explicitly."
            .format(system, machine),
            file=sys.stderr,
        )
        sys.exit(2)

    clang_basename = "clang+llvm-{}-{}".format(args.llvm_version, release_arch)

    tempdir = args.tempdir.resolve()
    source_dir = tempdir / "llvm.src"
    clang_topdir = tempdir / "clang"
    clang_dir = clang_topdir / clang_basename
    download_success_file = tempdir / "download.success"

    required_programs = ["cmake", "ninja", "git", "curl", "tar", "zstd"]
    if args.clang:
        required_programs.extend(["clang", "clang++"])
    require_programs(required_programs)

    ensure_dir(tempdir)

    if args.force_redownload:
        download_success_file.unlink(missing_ok=True)

    if not download_success_file.exists():
        clone_llvm_source(source_dir, args.llvm_version)
        if args.libcxx and not args.clang:
            clang_url = (
                "https://github.com/llvm/llvm-project/releases/download/llvmorg-{}/{}.tar.xz"
                .format(
                    args.llvm_version,
                    clang_basename,
                ))
            download_extract_clang_distribution(clang_url, clang_dir)
        download_success_file.touch()
    else:
        if args.libcxx and not args.clang and not clang_dir.exists():
            print(
                "{} is missing; rerun with --force-redownload or --clang.".
                format(clang_dir),
                file=sys.stderr,
            )
            sys.exit(2)

    if args.clang:
        build_clang_release(
            tempdir=tempdir,
            source_dir=source_dir,
            clang_topdir=clang_topdir,
            clang_dir=clang_dir,
            clang_basename=clang_basename,
            force_rebuild=args.clang_force_rebuild,
        )

    if args.clang or args.libcxx:
        if not clang_dir.exists():
            print("Expected clang install directory at {}".format(clang_dir),
                  file=sys.stderr)
            sys.exit(2)

    for libcxx_variant in args.libcxx:
        build_libcxx_variant(
            variant=libcxx_variant,
            source_dir=source_dir,
            tempdir=tempdir,
            clang_dir=clang_dir,
            llvm_version=args.llvm_version,
            release_arch=release_arch,
        )


if __name__ == "__main__":
    main()
