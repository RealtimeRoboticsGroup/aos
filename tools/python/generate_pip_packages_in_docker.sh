#!/bin/bash

# This script runs inside of a docker container to download the wheels in our
# requirements lock file. If necessary, this script will also build wheels for
# packages that are only available in source form. If a wheel is built from
# source, it will be made more hermetic with the "auditwheel" tool. That tool
# grafts system libraries into the wheel itself as per PEP600.
#
# This file is largely inspired by the manylinux demo:
# https://github.com/pypa/python-manylinux-demo/blob/master/travis/build-wheels.sh
#
# This file is more complicated than the demo largely because of a bug in the
# "auditwheel" tool. It can't deal with already-fixed packages. Once that's
# fixed, I think we can simplify this script quite a bit.
# https://github.com/pypa/auditwheel/issues/394

set -o errexit
set -o nounset
set -o pipefail

readonly PLAT="$1"
readonly ARCH="$2"
readonly CALLER_ID="$3"

readonly WHEELHOUSE_MIRROR_URL="https://realtimeroboticsgroup.org/build-dependencies/wheelhouse/simple"

# Try to make the wheels reproducible by telling them we're in 1980.
# Unfortunately, this is insufficient due to a pip bug.
# https://github.com/pypa/pip/issues/9604
export SOURCE_DATE_EPOCH=315532800

SCRIPT_DIR="$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")"
readonly SCRIPT_DIR

clean_up() {
  chown -R "${CALLER_ID}:${CALLER_ID}" "${SCRIPT_DIR}"
}

trap clean_up EXIT

# Clean up and prepare the final wheelhouse directory.
# Pip checks that the cache directory (/root/.cache/pip) is owned by the current
# user (root). Since .pip_cache was created by the host caller, it is owned by
# the host user inside the container, causing pip to disable the cache with a
# warning. We change the ownership to root:root inside the container to satisfy
# pip. The EXIT trap (clean_up) will restore ownership back to the host user
# before exiting.
if [ -d "${SCRIPT_DIR}/.pip_cache" ]; then
  chown -R root:root "${SCRIPT_DIR}/.pip_cache"
fi
rm -rf "${SCRIPT_DIR}"/wheelhouse
mkdir -p "${SCRIPT_DIR}"/wheelhouse

# Loop over the Python versions we support.
for python_version in "3.10" "3.13"; do
  echo "========================================================================"
  echo "Building wheels for Python ${python_version}"
  echo "========================================================================"

  # Point /opt/python and /install to the correct python version directory.
  python_dir="/opt/python${python_version}"
  rm -rf /opt/python /install
  ln -s "${python_dir}" /opt/python
  ln -s "${python_dir}" /install

  python_bin="${python_dir}/bin/python3"

  # Clean up temporary folders from previous iterations.
  rm -rf \
    "${SCRIPT_DIR}"/venv \
    "${SCRIPT_DIR}"/wheelhouse_tmp

  mkdir "${SCRIPT_DIR}"/venv
  pushd "${SCRIPT_DIR}"/venv

  "${python_bin}" -m venv venv
  source venv/bin/activate

  PIP_BIN=(pip)

  # Might be useful for debugging.
  "${PIP_BIN[@]}" --version

  # Get wheels for everything. Everything is stored in a temporary wheelhouse in
  # case we need to run the "auditwheel" tool against them.
  "${PIP_BIN[@]}" install wheel
  "${PIP_BIN[@]}" wheel \
    --no-deps \
    -r "${SCRIPT_DIR}/requirements.lock.txt" \
    -w "${SCRIPT_DIR}/wheelhouse_tmp/" \
    --index-url="${WHEELHOUSE_MIRROR_URL}" \
    --extra-index-url=https://pypi.org/simple \
    --prefer-binary \
    --timeout=500 \
    | tee /tmp/pip-wheel.log

  "${PIP_BIN[@]}" wheel \
    --no-deps \
    -r "${SCRIPT_DIR}/aos_requirements.lock.txt" \
    -w "${SCRIPT_DIR}/wheelhouse_tmp/" \
    --index-url="${WHEELHOUSE_MIRROR_URL}" \
    --extra-index-url=https://pypi.org/simple \
    --prefer-binary \
    --timeout=500 \
    | tee -a /tmp/pip-wheel.log

  # Find the list of packages that were built from source.
  # We need to suppress the exit code of grep here because it returns non-zero if
  # we have no source packages.
  source_packages=($( \
    (grep -o '^\s\+Building wheel for [-_.a-zA-Z0-9]\+' /tmp/pip-wheel.log || :) \
    | awk '{print $4}' \
    | sort -u \
    ))

  # Let the user know which packages we built ourselves.
  echo "The following packages were built from source based on pip's output."
  for package in "${source_packages[@]}"; do
    echo " - ${package}"
  done
  if ((${#source_packages[@]} == 0)); then
    echo " (no source packages)"
  fi

  # Find the list of actual wheel filenames we built.
  wheels_built_from_source=()
  for package in "${source_packages[@]}"; do
    # Extract lines that look roughly like so:
    # Created wheel for orderedset: filename=orderedset-2.0.3-cp39-cp39-linux_x86_64.whl size=382564 sha256=70fd9e3ab45cf737048b757ba219adf11a691963fbb88c9f16f6eef3866239a9
    log_line="$(grep -o "Created wheel for ${package}: filename=[^ ]\\+" /tmp/pip-wheel.log)"
    filename="$(cut -d= -f2 <<<"${log_line}")"
    wheels_built_from_source+=("${filename}")
  done

  # Make the wheels we built more hermetic. The auditwheel tool will graft system
  # libraries into the wheel itself. The list of system libraries that will not
  # get grafted is here:
  # https://peps.python.org/pep-0599/#the-manylinux2014-policy
  "${PIP_BIN[@]}" install auditwheel
  for wheel in "${wheels_built_from_source[@]}"; do
    wheel_path="${SCRIPT_DIR}/wheelhouse_tmp/${wheel}"

    # Skip the pygobject wheel for now. I have no idea why, but repairing it will
    # prevent it from finding certain files. Possibly some issue with paths
    # relative to the .so file.
    # TODO(phil): Figure out what's wrong with the repaired wheel.
    if [[ "${wheel}" == PyGObject-*.whl ]]; then
      echo "Not repairing ${wheel} because of issues."
      cp "${wheel_path}" "${SCRIPT_DIR}"/wheelhouse/
      continue
    fi

    echo "Repairing wheel ${wheel}"
    if ! auditwheel show "${wheel_path}"; then
      echo "Assuming ${wheel} is a non-platform wheel. Skipping."
      cp "${wheel_path}" "${SCRIPT_DIR}"/wheelhouse/
      continue
    fi
    auditwheel repair \
      --plat "${PLAT}_${ARCH}" \
      --only-plat \
      -w "${SCRIPT_DIR}"/wheelhouse/ \
      "${wheel_path}"
  done

  # Copy the downloaded wheels into the final wheelhouse too.
  downloaded_wheels=($(grep '^Saved [^ ]\+\.whl$' /tmp/pip-wheel.log \
    | awk '{print $2}'))
  if ((${#downloaded_wheels[@]} > 0)); then
    cp "${downloaded_wheels[@]}" "${SCRIPT_DIR}"/wheelhouse/
  fi

  deactivate
  popd
done
