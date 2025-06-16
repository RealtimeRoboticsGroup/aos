import argparse
import re
import sys
from pathlib import Path

import pkginfo


def normalize(name: str) -> str:
    """Normalizes a package name.

    See the user guide for details:
    https://packaging.python.org/en/latest/specifications/name-normalization/#name-normalization
    """
    return re.sub(r"[-_.]+", "-", name).lower()

def create_symlinks(directory: Path) -> None:
    simple = directory / "simple"

    for wheel_path in directory.glob("*.whl"):
        wheel = pkginfo.Wheel(wheel_path)

        project_dir = simple / normalize(wheel.name)
        project_dir.mkdir(parents=True, exist_ok=True)

        organized_wheel = project_dir / wheel_path.name
        organized_wheel.symlink_to(wheel_path)

def main(argv):
    parser = argparse.ArgumentParser()
    parser.add_argument("--directory")
    args = parser.parse_args(argv[1:])

    create_symlinks(Path(args.directory))

if __name__ == "__main__":
    sys.exit(main(sys.argv))
