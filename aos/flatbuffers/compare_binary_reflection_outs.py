#!/usr/bin/env python3
import sys
import hashlib
import filecmp


def main():
    if len(sys.argv) < 5:
        print(
            "Usage: compare_binary_reflection_outs.py <file1> <file2> <file3> <file4>"
        )
        sys.exit(1)

    files = sys.argv[1:5]

    print("Comparing bfbs hashes:")
    for filepath in files:
        hasher = hashlib.sha256()
        with open(filepath, "rb") as f:
            while chunk := f.read(8192):
                hasher.update(chunk)
        print(f"{hasher.hexdigest()}  {filepath}")

    # Compare files to verify they are binary identical
    file1 = files[0]
    for other_file in files[1:]:
        if not filecmp.cmp(file1, other_file, shallow=False):
            print(f"ERROR: {file1} and {other_file} are not binary identical!",
                  file=sys.stderr)
            sys.exit(1)

    print("PASS: All 4 binary reflection schemas (.bfbs) are identical!")


if __name__ == "__main__":
    main()
