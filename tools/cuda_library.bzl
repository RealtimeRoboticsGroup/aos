"""Macro wrapper for cuda_library that adds GPU platform constraints."""

load("@rules_cuda//cuda:defs.bzl", _cuda_library = "cuda_library")

def cuda_library(name, target_compatible_with = None, **kwargs):
    """Wrapper for rules_cuda's cuda_library with automatic GPU platform constraints.

    This macro automatically adds the '//tools/platforms/gpu:nvidia' constraint
    to ensure CUDA targets are only built on GPU-capable platforms. For non-GPU
    platforms (like roborio), the no-op CUDA toolchain will be selected, and
    these targets will be skipped due to target incompatibility.

    Args:
        name: The name of the target.
        target_compatible_with: Additional target compatibility constraints.
            The GPU constraint is added automatically.
        **kwargs: Other arguments passed to the underlying cuda_library rule.
    """

    # Build the target_compatible_with list with GPU constraint
    constraints = ["//tools/platforms/gpu:nvidia"]
    if target_compatible_with:
        constraints.extend(target_compatible_with)

    _cuda_library(
        name = name,
        target_compatible_with = constraints,
        **kwargs
    )
