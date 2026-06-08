#!/bin/bash

set -e
set -u
set -o pipefail

# We disable writing .pyc files here so that the invocation is more
# deterministic. If we get a corrupted .pyc file (for some reason) in the
# .runfiles directory the corresponding Python invocation would crash with an
# EOFError. You can try this by calling truncate(1) on a .pyc file and running
# your Python script.
# In the bazel sandbox none of the .pyc files are preserved anyway.
# Sandboxing also means that Python's entire standard library got cached which
# normally doesn't happen. That can lead to higher memory usage during the
# individual build steps.
export PYTHONDONTWRITEBYTECODE=1

# Find the path that contains the Python runtime. It's not always obvious. For
# example in a genrule the Python runtime is in the runfiles folder of the
# tool, not of the genrule.
# TODO(philipp): Is there a better way to do this?
PYTHON_BIN=""

if [[ -n "${RUNFILES_DIR:-}" ]]; then
  path_plus="${RUNFILES_DIR}/rules_python++python+python_3_10_x86_64-unknown-linux-gnu"
  path_tilde="${RUNFILES_DIR}/rules_python~~python~python_3_10_x86_64-unknown-linux-gnu"

  if [[ -d "${path_plus}" ]]; then
    path="${path_plus}"
  elif [[ -d "${path_tilde}" ]]; then
    path="${path_tilde}"
  fi

  if [[ -d "${path}" ]]; then
    PYTHON_BIN="$path"/bin/python3
    LD_LIBRARY_PATH=":${path}/lib"
    LD_LIBRARY_PATH+=":${path}/../amd64_debian_sysroot/lib/x86_64-linux-gnu/"
    LD_LIBRARY_PATH+=":${path}/../amd64_debian_sysroot/usr/lib/x86_64-linux-gnu/"
    LD_LIBRARY_PATH+=":${path}/../amd64_debian_sysroot/usr/lib/"
    LD_LIBRARY_PATH+=":${path}/../amd64_debian_sysroot/usr/lib/x86_64-linux-gnu/gvfs/"
    # Try different possible repository names due to bzlmod / workspace variations
    for suffix in "rules_python++pip+aos_pip_deps_310_nvidia_nccl_cu12" "rules_python~~pip~aos_pip_deps_310_nvidia_nccl_cu12" "pip_deps_nvidia_nccl_cu12"; do
      if [[ -e "${path}/../${suffix}" ]]; then
        LD_LIBRARY_PATH+=":${path}/../${suffix}/site-packages/nvidia/nccl/lib/"
        break
      fi
    done
    export LD_LIBRARY_PATH
  fi
fi

if [[ -z "$PYTHON_BIN" ]]; then
  paths="${PYTHONPATH:-}"
  for path in ${paths//:/ }; do
    if [[ "$path" == *.runfiles/python_3_10_x86_64-unknown-linux-gnu ]]; then
      PYTHON_BIN="$path"/bin/python3
      LD_LIBRARY_PATH=":${path}/lib"
      LD_LIBRARY_PATH+=":${path}/../amd64_debian_sysroot/lib/x86_64-linux-gnu/"
      LD_LIBRARY_PATH+=":${path}/../amd64_debian_sysroot/usr/lib/x86_64-linux-gnu/"
      LD_LIBRARY_PATH+=":${path}/../amd64_debian_sysroot/usr/lib/"
      LD_LIBRARY_PATH+=":${path}/../amd64_debian_sysroot/usr/lib/x86_64-linux-gnu/gvfs/"
      for suffix in "rules_python++pip+aos_pip_deps_310_nvidia_nccl_cu12" "rules_python~~pip~aos_pip_deps_310_nvidia_nccl_cu12" "pip_deps_nvidia_nccl_cu12"; do
        if [[ -e "${path}/../${suffix}" ]]; then
          LD_LIBRARY_PATH+=":${path}/../${suffix}/site-packages/nvidia/nccl/lib/"
          break
        fi
      done
      export LD_LIBRARY_PATH
      break
    fi
  done
fi

if [[ -z "$PYTHON_BIN" ]]; then
  echo "Could not find Python base path." >&2
  echo "More sophisticated logic may be needed." >&2
  exit 1
fi

# Locate the runfiles directory and find any bundled NVIDIA CUDA/cuDNN components.
RUNFILES_ROOT=""
if [[ -n "${RUNFILES_DIR:-}" && -d "${RUNFILES_DIR}" ]]; then
  RUNFILES_ROOT="${RUNFILES_DIR}"
elif [[ -n "${path:-}" && -d "${path}/.." ]]; then
  RUNFILES_ROOT=$(cd "${path}/.." && pwd)
fi

CUDA_DATA_DIR=""
if [[ -n "${RUNFILES_ROOT}" ]]; then
  # Enable nullglob to avoid literal patterns when no files match
  shopt -s nullglob
  for dir in "${RUNFILES_ROOT}"/*nvidia_*; do
    pkg_path="${dir}/site-packages/nvidia"
    if [[ -d "${pkg_path}" ]]; then
      for subpkg in "${pkg_path}"/*; do
        if [[ -d "${subpkg}" ]]; then
          if [[ -d "${subpkg}/lib" ]]; then
            if [[ -z "${LD_LIBRARY_PATH:-}" ]]; then
              LD_LIBRARY_PATH="${subpkg}/lib"
            else
              LD_LIBRARY_PATH="${subpkg}/lib:${LD_LIBRARY_PATH}"
            fi
          fi
          if [[ -d "${subpkg}/bin" ]]; then
            PATH="${subpkg}/bin:${PATH}"
          fi
          if [[ -d "${subpkg}/nvvm/libdevice" ]]; then
            CUDA_DATA_DIR="${subpkg}"
          fi
        fi
      done
    fi
  done
  shopt -u nullglob

  if [[ -n "${LD_LIBRARY_PATH:-}" ]]; then
    export LD_LIBRARY_PATH
  fi
  if [[ -n "${PATH:-}" ]]; then
    export PATH
  fi
fi

# Configure XLA_FLAGS. Fall back to /usr/lib/cuda if no bundled libdevice is found.
current_xla_flags="${XLA_FLAGS:-}"
xla_data_dir="/usr/lib/cuda"
if [[ -n "${CUDA_DATA_DIR:-}" ]]; then
  xla_data_dir="${CUDA_DATA_DIR}"
fi

new_xla_flags=""
for flag in ${current_xla_flags}; do
  if [[ "$flag" != --xla_gpu_cuda_data_dir=* ]]; then
    new_xla_flags="${new_xla_flags} ${flag}"
  fi
done

export XLA_FLAGS="--xla_gpu_cuda_data_dir=${xla_data_dir}${new_xla_flags}"

# Prevent Python from importing the host's installed packages.
# We symlink the python binary to the current directory (which is the venv bin dir)
# so that it picks up pyvenv.cfg and the site-packages from the venv.
# This ensures that bazel.pth is loaded and runfiles are initialized correctly.
ln -sf "$PYTHON_BIN" "$(dirname $0)/python3"

exec "$(dirname $0)/python3" "$@"
