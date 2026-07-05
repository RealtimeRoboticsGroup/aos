#!/bin/bash

set -e
set -u
set -o pipefail

export PYTHONDONTWRITEBYTECODE=1

readonly PYTHON_VER="3_13"
readonly SHORT_VER="313"

# Detect RUNFILES_DIR if not set.
if [[ -z "${RUNFILES_DIR:-}" ]]; then
  if [[ "$PWD" == *.runfiles/* ]]; then
    export RUNFILES_DIR="${PWD%%.runfiles/*}.runfiles"
  fi
fi

# Locate the Python interpreter path.
find_python_interpreter() {
  # 1. Try to find in runfiles first
  if [[ -n "${RUNFILES_DIR:-}" ]]; then
    for path in \
      "${RUNFILES_DIR}/rules_python++python+python_${PYTHON_VER}_x86_64-unknown-linux-gnu" \
      "${RUNFILES_DIR}/rules_python~~python~python_${PYTHON_VER}_x86_64-unknown-linux-gnu" \
      "${RUNFILES_DIR}/python_${PYTHON_VER}_x86_64-unknown-linux-gnu"; do
      if [[ -d "${path}" ]]; then
        echo "${path}"
        return 0
      fi
    done
  fi

  # 2. Try to find in PYTHONPATH next
  local paths="${PYTHONPATH:-}"
  for path in ${paths//:/ }; do
    if [[ "$path" == *.runfiles/python_${PYTHON_VER}_x86_64-unknown-linux-gnu ]]; then
      if [[ -d "${path}" ]]; then
        echo "${path}"
        return 0
      fi
    fi
  done

  # 3. Fallback for bazel run where RUNFILES_DIR / PYTHONPATH might not be propagated
  local workspace_dir="${BUILD_WORKSPACE_DIRECTORY:-}"
  if [[ -z "${workspace_dir}" ]]; then
    if [[ -f "WORKSPACE" || -f "MODULE.bazel" ]]; then
      workspace_dir="."
    fi
  fi
  if [[ -n "${workspace_dir}" ]]; then
    for suffix in \
      "rules_python++python+python_${PYTHON_VER}_x86_64-unknown-linux-gnu" \
      "rules_python~~python~python_${PYTHON_VER}_x86_64-unknown-linux-gnu" \
      "python_${PYTHON_VER}_x86_64-unknown-linux-gnu"; do
      local found_path
      found_path=$(find "${workspace_dir}/bazel-bin" -type d -name "${suffix}" -print -quit 2>/dev/null)
      if [[ -d "${found_path}" ]]; then
        echo "${found_path}"
        return 0
      fi
    done
  fi

  return 1
}

BASE_PATH=$(find_python_interpreter)
if [[ -z "${BASE_PATH}" ]]; then
  echo "Could not find Python ${PYTHON_VER} base path." >&2
  exit 1
fi

PYTHON_BIN="${BASE_PATH}/bin/python3"
LD_LIBRARY_PATH=":${BASE_PATH}/lib"
LD_LIBRARY_PATH+=":${BASE_PATH}/../amd64_debian_sysroot/lib/x86_64-linux-gnu/"
LD_LIBRARY_PATH+=":${BASE_PATH}/../amd64_debian_sysroot/usr/lib/x86_64-linux-gnu/"
LD_LIBRARY_PATH+=":${BASE_PATH}/../amd64_debian_sysroot/usr/lib/"
LD_LIBRARY_PATH+=":${BASE_PATH}/../amd64_debian_sysroot/usr/lib/x86_64-linux-gnu/gvfs/"

# Try different possible repository names due to bzlmod / workspace variations for nvidia/nccl
for suffix in \
  "rules_python++pip+aos_pip_deps_${SHORT_VER}_nvidia_nccl_cu12" \
  "rules_python~~pip~aos_pip_deps_${SHORT_VER}_nvidia_nccl_cu12" \
  "pip_deps_nvidia_nccl_cu12"; do
  if [[ -e "${BASE_PATH}/../${suffix}" ]]; then
    LD_LIBRARY_PATH+=":${BASE_PATH}/../${suffix}/site-packages/nvidia/nccl/lib/"
    break
  fi
done

# Locate the runfiles directory and find any bundled NVIDIA CUDA/cuDNN components.
RUNFILES_ROOT=""
if [[ -n "${RUNFILES_DIR:-}" && -d "${RUNFILES_DIR}" ]]; then
  RUNFILES_ROOT="${RUNFILES_DIR}"
elif [[ -d "${BASE_PATH}/.." ]]; then
  RUNFILES_ROOT=$(cd "${BASE_PATH}/.." && pwd)
fi

setup_cuda_environment() {
  if [[ -z "${RUNFILES_ROOT}" ]]; then
    return
  fi

  local cuda_data_dir=""
  shopt -s nullglob
  for dir in "${RUNFILES_ROOT}"/*nvidia_*; do
    local pkg_path="${dir}/site-packages/nvidia"
    if [[ -d "${pkg_path}" ]]; then
      for subpkg in "${pkg_path}"/*; do
        if [[ -d "${subpkg}" ]]; then
          if [[ -d "${subpkg}/lib" ]]; then
            LD_LIBRARY_PATH="${subpkg}/lib:${LD_LIBRARY_PATH}"
          fi
          if [[ -d "${subpkg}/bin" ]]; then
            PATH="${subpkg}/bin:${PATH}"
          fi
          if [[ -d "${subpkg}/nvvm/libdevice" ]]; then
            cuda_data_dir="${subpkg}"
          fi
        fi
      done
    fi
  done
  shopt -u nullglob

  # Configure XLA_FLAGS. Fall back to /usr/lib/cuda if no bundled libdevice is found.
  local current_xla_flags="${XLA_FLAGS:-}"
  local xla_data_dir="/usr/lib/cuda"
  if [[ -n "${cuda_data_dir}" ]]; then
    xla_data_dir="${cuda_data_dir}"
  fi

  local new_xla_flags=""
  for flag in ${current_xla_flags}; do
    if [[ "$flag" != --xla_gpu_cuda_data_dir=* ]]; then
      new_xla_flags="${new_xla_flags} ${flag}"
    fi
  done

  export XLA_FLAGS="--xla_gpu_cuda_data_dir=${xla_data_dir}${new_xla_flags}"
  export PATH
}

setup_cuda_environment
export LD_LIBRARY_PATH

# Prevent Python from importing the host's installed packages.
ln -sf "$PYTHON_BIN" "$(dirname $0)/python3"
exec "$(dirname $0)/python3" "$@"
