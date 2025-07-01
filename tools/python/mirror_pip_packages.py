"""This script mirrors our pip package dependencies.

This script looks at the requirements.lock.txt file and generate a wheel for
each entry. Those wheels are then mirrored.

See tools/python/README.md for some more information.
"""

# TODO(philsc): Remove this whole file and use rules_uv instead.

import argparse
import hashlib
import json
import os
import pwd
import re
import subprocess
import sys
import tarfile
from pathlib import Path
from typing import List, Optional, Tuple

import requests
from pkginfo import Wheel

PLAT = "manylinux_2_34"
ARCH = "x86_64"
WHEELHOUSE_MIRROR_URL = "https://realtimeroboticsgroup.org/build-dependencies/wheelhouse/simple"
WHEELHOUSE_GCS_URL = "gs://austin-vpn-build-dependencies/wheelhouse/simple"


def normalize_name(name: str) -> str:
    """Normalizes a package name so it's consistent across all use cases.

    This follows the upstream spec:
    https://packaging.python.org/en/latest/specifications/name-normalization/#name-normalization

    pip is really inconsistent about using real package names vs. whatever
    users typed into the requirements file. It feels random.
    Everything is lower-cased and dashes are replaced by underscores.

    Args:
        name: The name to normalize.

    Returns:
        The normalized name.
    """
    return re.sub(r"[-_.]+", "-", name).lower()


def compute_sha256(data: bytes) -> str:
    """Computes the sha256 checksum of a bytes sequence.

    Args:
        data: The bytes to checksum.

    Returns:
        The hex representation of the checksum.
    """
    hasher = hashlib.sha256()
    hasher.update(data)
    return hasher.hexdigest()


def compute_file_sha256(filename: Path) -> str:
    """Computes the sha256 checksum of the content of a file.

    Args:
        filename: The file to checksum.

    Returns:
        The hex representation of the checksum.
    """
    return compute_sha256(filename.read_bytes())


def wheel_is_already_uploaded(wheel: Path, wheel_url: str) -> Tuple[bool, str]:
    """Searches for this wheel on our internal mirror.

    Since we can't build wheels reproducibly, our best option is to check
    whether this wheel already exists on the mirror. If it does, we can skip
    uploading it.

    Args:
        wheel: The wheel to search for on the mirror.
        wheel_url: The URL where the wheel is expected if it exists on the mirror.

    Returns:
        A two-tuple. The first value is a boolean that signifies whether the
        wheel was found on the mirror. The second value is a string. If the
        wheel was not found on the mirror, this is an empty string. Otherwise,
        this string contains the sha256 checksum of the wheel found on the
        mirror.
    """
    print(f"Checking if {wheel.name} is already uploaded: ", end="")
    request = requests.head(wheel_url)

    if request.status_code == 200:
        print("yes")
        return True
    if request.status_code == 404:
        print("no")
        return False

    raise RuntimeError(
        f"Don't know what to do with status code {request.status_cdoe} when trying to get {wheel_url}"
    )


def copy_to_gcs(filename: Path) -> None:
    """Copies the specified wheel to GCS.

    Args:
        filename: The path to the tarball to be uploaded.
    """

    info = Wheel(filename)
    normalized_name = normalize_name(info.name)
    gcs_path = f"{WHEELHOUSE_GCS_URL}/{normalized_name}/{filename.name}"

    command = ["gsutil", "cp", filename, gcs_path]

    print(command)


def main(argv: List[str]) -> Optional[int]:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-f",
        "--force",
        action="store_true",
        help=("If set, ignores packages we have already uploaded and "
              "possibly overwrite them with the just-built ones. Use with "
              "extreme caution! This may easily cause issues with building "
              "older commits. Use this only if you know what you're doing."))
    args = parser.parse_args(argv[1:])

    root_dir = Path(os.environ["BUILD_WORKSPACE_DIRECTORY"])
    caller = os.getenv("SUDO_USER") or os.environ["USER"]
    caller_id = pwd.getpwnam(caller).pw_uid

    python_dir = root_dir / "tools" / "python"

    container_tag = f"pip-compile:{caller}"

    subprocess.run([
        "docker",
        "build",
        "--file=generate_pip_packages.Dockerfile",
        f"--tag={container_tag}",
        ".",
    ],
                   cwd=python_dir,
                   check=True)

    ## Run the wheel generation script inside the docker container provided by
    ## the pypa/manylinux project.
    ## https://github.com/pypa/manylinux/
    #subprocess.run([
    #    "docker",
    #    "run",
    #    "-it",
    #    "--net=host",
    #    "-v",
    #    f"{python_dir}:/opt/build/",
    #    container_tag,
    #    "/opt/build/generate_pip_packages_in_docker.sh",
    #    PLAT,
    #    ARCH,
    #    str(caller_id),
    #],
    #               check=True)

    # Get the list of wheels we downloaded form pypi.org or built ourselves.
    wheelhouse = python_dir / "wheelhouse"
    wheels = wheelhouse.glob("*.whl")

    # Assemble the override list. This list will tell rules_python to download
    # from our mirror instead of pypi.org.
    wheels_to_be_uploaded = []
    override_information = {}
    for wheel in sorted(wheels):
        info = Wheel(wheel)
        normalized_name = normalize_name(info.name)

        wheel_url = f"{WHEELHOUSE_MIRROR_URL}/{normalized_name}/{wheel.name}"

        # Check if we already have the wheel uploaded. We can skip uploading
        # that.
        wheel_found = wheel_is_already_uploaded(wheel, wheel_url)

        if args.force:
            if wheel_found:
                print(
                    f"WARNING: The next upload may change sha256 for {wheel}!"
                )
            wheels_to_be_uploaded.append(wheel)
        elif not wheel_found:
            wheels_to_be_uploaded.append(wheel)

    print(f"We need to upload {len(wheels_to_be_uploaded)} wheels:")
    for wheel in wheels_to_be_uploaded:
        print(wheel)

    # Upload the wheels if requested.
    for wheel in wheels_to_be_uploaded:
        copy_to_gcs(wheel)


if __name__ == "__main__":
    sys.exit(main(sys.argv))
