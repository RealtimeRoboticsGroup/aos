# LLVM helper scripts

## `build_from_llvm.py`

Use `tools/helpers/build_from_llvm.py` to build LLVM release tarballs from
`llvm-project` source.

This is intended for occasional regeneration of the LLVM tarballs referenced by
AOS in `MODULE.bazel` and `WORKSPACE`.

### Prerequisites

Install these tools on the build machine:

- `git`
- `cmake`
- `ninja`
- `clang`
- `clang++`
- `curl`
- `tar`
- `zstd`

### Tarballs currently used by AOS

Today, AOS is configured to use these two LLVM tarballs:

- `clang+llvm-21.1.1-x86_64-linux-gnu-ubuntu-22.04.tar.zst`
- `clang+llvm-21.1.1-aarch64-linux-gnu.tar.zst`

Generate each tarball on a native host of that architecture.
Pieces of the aarch64 tarball are used for cross compiling for the ORIN.

## Build x86_64 tarball

Run on an `x86_64` Linux machine with **glibc ≥ 2.34** (e.g., Ubuntu 22.04 Jammy or newer):

```bash
./tools/helpers/build_from_llvm.py \
  --llvm-version 21.1.1 \
  --clang \
  --release-arch x86_64-linux-gnu-ubuntu-22.04 \
  --tempdir /tmp/llvm-build-x86_64 \
  --force-redownload \
  --clang-force-rebuild

mv /tmp/llvm-build-x86_64/LLVM-21.1.1-x86_64-linux-gnu-ubuntu-22.04.tar.zst \
   /tmp/llvm-build-x86_64/clang+llvm-21.1.1-x86_64-linux-gnu-ubuntu-22.04.tar.zst
```

Ubuntu 22.04 (glibc 2.35) is recommended for rebuilding for widest compatibility.

## Build aarch64 tarball

Run on an `aarch64` Linux machine:

```bash
./tools/helpers/build_from_llvm.py \
  --llvm-version 21.1.1 \
  --clang \
  --release-arch aarch64-linux-gnu \
  --tempdir /tmp/llvm-build-aarch64 \
  --force-redownload \
  --clang-force-rebuild

mv /tmp/llvm-build-aarch64/LLVM-21.1.1-aarch64-linux-gnu.tar.zst \
   /tmp/llvm-build-aarch64/clang+llvm-21.1.1-aarch64-linux-gnu.tar.zst
```

## Compute SHA256 checksums

```bash
sha256sum \
  /tmp/llvm-build-x86_64/clang+llvm-21.1.1-x86_64-linux-gnu-ubuntu-22.04.tar.zst \
  /tmp/llvm-build-aarch64/clang+llvm-21.1.1-aarch64-linux-gnu.tar.zst
```

Copy the hashes into `llvm_extra_distributions` in both:

- `MODULE.bazel`
- `WORKSPACE`

Then upload/rehost the tarballs and keep `llvm_alternative_sources` in sync if
needed.
