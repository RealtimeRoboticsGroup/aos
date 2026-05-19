#!/usr/bin/env python3
"""Reproducibly repack Alpine's `initramfs-virt` with our /init and a
handful of kernel modules dropped in.

The bazel CudaCompile actions on Darwin boot a vfkit microvm using
this initramfs to run the x86_64 CUDA host tools under Rosetta. The
inputs to that boot need to be byte-for-byte deterministic so that
their sha256 doesn't drift across re-publishes on the mirror.

This script:
  - Reads the stock `initramfs-virt` (cpio newc + gzip from Alpine).
  - Replaces /init with the supplied script.
  - Adds the supplied kernel modules under
    `lib/modules/<KVER>/kernel/fs/{,fuse/}`.
  - Writes a new cpio newc archive (uncompressed; pipe through `gzip -n`
    for the final artifact) with: mtime=0, uid=gid=0, nlink=1 for files,
    inode numbers allocated sequentially in sorted-by-name order. That
    set of normalisations is what `cpio --reproducible` does, plus we
    fix the name order ourselves so we don't depend on host iteration
    order in `find`.

cpio newc format (per `man 5 cpio`) is a 110-byte ASCII header per
entry followed by NUL-terminated name (padded to 4 bytes) and file
data (padded to 4 bytes). Stock Alpine initramfs uses only regular
files, directories, and symlinks (verified with `cpio -tv`), so we
don't need to round-trip device nodes, fifos, or sockets.
"""

from __future__ import annotations

import argparse
import dataclasses
import gzip
import io
import stat
import sys
from pathlib import Path

CPIO_MAGIC = b"070701"
TRAILER = b"TRAILER!!!"


@dataclasses.dataclass
class Entry:
    name: bytes  # without trailing NUL
    mode: int
    data: bytes  # file contents (or symlink target). Empty for dirs.

    def is_dir(self) -> bool:
        return stat.S_ISDIR(self.mode)


def _read_cpio(blob: bytes) -> list[Entry]:
    """Parse a cpio newc archive into a list of Entry objects."""
    entries: list[Entry] = []
    pos = 0
    n = len(blob)
    while pos < n:
        if blob[pos:pos + 6] != CPIO_MAGIC:
            raise ValueError(
                f"bad cpio magic at offset {pos}: {blob[pos:pos+6]!r}")
        hdr = blob[pos:pos + 110]
        pos += 110

        # All header numerics are 8-byte ASCII hex.
        def _h(i: int) -> int:
            return int(hdr[6 + i * 8:6 + (i + 1) * 8], 16)

        ino = _h(0)  # noqa: F841 — discarded; we re-number on write
        mode = _h(1)
        uid = _h(2)  # noqa: F841
        gid = _h(3)  # noqa: F841
        nlink = _h(4)  # noqa: F841
        mtime = _h(5)  # noqa: F841
        filesize = _h(6)
        # devmajor/devminor/rdevmajor/rdevminor/check ignored: we
        # don't emit device entries, and the stock initramfs doesn't
        # have any.
        namesize = _h(11)
        name = blob[pos:pos + namesize - 1]  # strip trailing NUL
        pos += namesize
        # Pad to 4-byte boundary after (110 + namesize).
        pad = (-(110 + namesize)) & 3
        pos += pad
        data = blob[pos:pos + filesize]
        pos += filesize
        # Pad data to 4-byte boundary.
        pos += (-filesize) & 3

        if name == TRAILER:
            break
        entries.append(Entry(name=name, mode=mode, data=data))
    return entries


def _write_cpio(entries: list[Entry]) -> bytes:
    """Serialise entries to a cpio newc archive.

    Entries are emitted in the order given; the caller is responsible
    for sorting if reproducibility across runs is desired.
    """
    out = io.BytesIO()

    def emit(name: bytes, mode: int, data: bytes, ino: int):
        # `nlink` must be >=2 for directories per the cpio spec, and is
        # ignored by the kernel's initramfs decoder for our purposes,
        # but most cpio tools complain otherwise.
        nlink = 2 if stat.S_ISDIR(mode) else 1
        namesize = len(name) + 1  # trailing NUL
        filesize = len(data)
        hdr = (
            CPIO_MAGIC + f"{ino:08x}".encode() + f"{mode:08x}".encode() +
            b"00000000"  # uid
            + b"00000000"  # gid
            + f"{nlink:08x}".encode() + b"00000000"  # mtime
            + f"{filesize:08x}".encode() + b"00000000"  # devmajor
            + b"00000000"  # devminor
            + b"00000000"  # rdevmajor
            + b"00000000"  # rdevminor
            + f"{namesize:08x}".encode() + b"00000000"  # check
        )
        assert len(hdr) == 110
        out.write(hdr)
        out.write(name)
        out.write(b"\x00")
        # Pad name+header to 4-byte boundary.
        out.write(b"\x00" * ((-(110 + namesize)) & 3))
        out.write(data)
        out.write(b"\x00" * ((-filesize) & 3))

    for i, e in enumerate(entries, start=1):
        emit(e.name, e.mode, e.data, ino=i)
    # Trailer entry must have a real header, mode 0, zero data; ino is
    # conventionally 0 but reproducible cpio uses a fixed value.
    emit(TRAILER, mode=0, data=b"", ino=0)
    # Pad whole archive to 512 bytes (matches GNU cpio's `-H newc`
    # behavior; the kernel doesn't care, but matching cpio's output
    # makes our archives drop-in compatible).
    total = out.tell()
    pad = (-total) & 511
    out.write(b"\x00" * pad)
    return out.getvalue()


def _make_dir_entries(path: bytes, existing: set[bytes]) -> list[Entry]:
    """Generate any missing intermediate directory entries for `path`."""
    new: list[Entry] = []
    parts = path.split(b"/")
    for i in range(1, len(parts)):
        d = b"/".join(parts[:i])
        if d and d not in existing:
            new.append(Entry(name=d, mode=stat.S_IFDIR | 0o755, data=b""))
            existing.add(d)
    return new


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--input",
                   required=True,
                   help="Path to the stock initramfs (cpio.gz).")
    p.add_argument("--init",
                   required=True,
                   help="Replacement /init script (mode 0755).")
    p.add_argument("--kernel-version",
                   required=True,
                   help="Alpine kernel version, used in the modules path.")
    p.add_argument("--module",
                   action="append",
                   default=[],
                   metavar="DEST=SRC",
                   help="Kernel module to drop in. DEST is relative to "
                   "lib/modules/<KVER>/kernel/, e.g. 'fs/fuse/fuse.ko'.")
    p.add_argument("--output",
                   required=True,
                   help="Output path for the repacked initramfs (gzipped).")
    args = p.parse_args()

    raw = gzip.decompress(Path(args.input).read_bytes())
    entries = _read_cpio(raw)

    # Build a name -> index map for in-place replacement; new entries
    # are appended.
    by_name = {e.name: i for i, e in enumerate(entries)}

    # Replace /init.
    init_data = Path(args.init).read_bytes()
    init_entry = Entry(name=b"init", mode=stat.S_IFREG | 0o755, data=init_data)
    if b"init" in by_name:
        entries[by_name[b"init"]] = init_entry
    else:
        entries.append(init_entry)

    # Add kernel modules with any required intermediate dirs. Replace
    # in place if the name already exists in the input archive (Alpine
    # ships some of these in its stock initramfs).
    existing_dirs = {e.name for e in entries if e.is_dir()}
    kver = args.kernel_version
    mod_root = f"lib/modules/{kver}/kernel".encode()
    for spec in args.module:
        dest_rel, _, src = spec.partition("=")
        if not src:
            sys.exit(f"--module needs DEST=SRC, got: {spec!r}")
        dest = mod_root + b"/" + dest_rel.encode()
        entries.extend(_make_dir_entries(dest, existing_dirs))
        new_entry = Entry(name=dest,
                          mode=stat.S_IFREG | 0o644,
                          data=Path(src).read_bytes())
        if dest in by_name:
            entries[by_name[dest]] = new_entry
        else:
            entries.append(new_entry)
            by_name[dest] = len(entries) - 1

    # Sort by name. GNU cpio uses byte-sorted order with --reproducible
    # only when the input filenames are pre-sorted; sorting here gives
    # us byte-identical output regardless of input traversal order.
    entries.sort(key=lambda e: e.name)

    cpio = _write_cpio(entries)
    # gzip with mtime=0 and no original filename for byte-determinism.
    gz = gzip.compress(cpio, compresslevel=9, mtime=0)
    Path(args.output).write_bytes(gz)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
