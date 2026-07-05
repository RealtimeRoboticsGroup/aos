# AOS's Python setup

## How to depend on pip packages

AOS divides its pip dependencies into two separate repositories (hubs) to support
building subsets of the repository:

1. `@pip_deps`: For core and general-purpose third-party packages (e.g. `numpy`, `scipy`, `matplotlib`).
2. `@aos_pip_deps`: For AOS-specific packages (e.g. `pygobject`).

You can depend on packages in these hubs like so:

```python
py_binary(
    name = "bin",
    srcs = ["bin.py"],
    deps = [
        "@pip_deps//numpy",
        "@aos_pip_deps//pygobject",
    ],
)
```

The labels are "normalized". That means the entries in the requirements files may not be usable as-is. When you know the name of the package, apply the following transformations:

1. Make the name lower-case.
2. Replace all dots and dashes with underscores.

For example:
- `Jinja2` becomes `@pip_deps//jinja2`.
- `absl-py` becomes `@pip_deps//absl_py`.
- `Flask-SQLAlchemy` becomes `@pip_deps//flask_sqlalchemy`.
- `ruamel.yaml` becomes `@pip_deps//ruamel_yaml`.

## How to add new pip packages

1.  Add the new package you're interested in to `tools/python/requirements.txt` (core) or `tools/python/aos_requirements.txt` (AOS-specific).
2.  Run the lock file generation script for the corresponding requirement set:

    To update core dependencies:
    ```bash
    bazel run --run_under=//tools/python:update_helper //tools/python:requirements.update
    ```

    To update AOS dependencies:
    ```bash
    bazel run --run_under=//tools/python:update_helper //tools/python:aos_requirements.update
    ```

## How to make buildkite happy with new pip packages

In order for buildkite to be able to use new pip packages, they have to be
mirrored on aos infrastructure.

1.  Follow the above procedure for adding new pip packages if not already done.
2.  Run the mirroring script:
    ```bash
    bazel run //tools/python:mirror_pip_packages
    ```
    This compiles and mirrors the wheels for both Python 3.10 and Python 3.13. This assumes that you have access to the correct GCS bucket.
