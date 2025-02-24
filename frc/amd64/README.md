# Building sysroots

The scripts in here will build both the Debian and Ubuntu sysroots used to compile against.
After building, compress them, upload them to realtimeroboticsgroup.org/build-dependencies/, and update the WORKSPACE file.

## Debian Bookworm

```
./build_rootfs.py
```

## Ubuntu Jammy

```
./build_rootfs_ubuntu.py
```

## Recommended compression

zstd does about as good or better than xz for compression, but decompresses like 10x faster.

```
zstd --ultra -21 -T0 2025-02-22-bookworm-amd64-nvidia-rootfs.tar
```
