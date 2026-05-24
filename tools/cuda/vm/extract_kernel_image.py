#!/usr/bin/env python3
"""Extract the raw arm64 kernel Image from Alpine's `vmlinuz-virt`.

Alpine ships an EFI-stub kernel where the actual `Image` payload is
embedded as a gzip stream. vfkit can only boot the raw uncompressed
kernel, so we scan for the first valid gzip member and decompress it.

Deterministic: the input bytes uniquely determine the output bytes.
"""

from __future__ import annotations

import argparse
import sys
import zlib
from pathlib import Path


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--input", required=True)
    p.add_argument("--output", required=True)
    args = p.parse_args()

    data = Path(args.input).read_bytes()
    for i in range(len(data) - 2):
        # gzip magic + deflate method byte.
        if data[i:i + 3] != b"\x1f\x8b\x08":
            continue
        try:
            decompressor = zlib.decompressobj(wbits=31)
            raw = decompressor.decompress(data[i:])
            if not decompressor.eof:
                continue
            Path(args.output).write_bytes(raw)
            print(f"extracted {len(raw)} bytes at offset {i}", file=sys.stderr)
            return 0
        except zlib.error:
            continue
    sys.exit("no decompressible gzip stream found in input")


if __name__ == "__main__":
    raise SystemExit(main())
