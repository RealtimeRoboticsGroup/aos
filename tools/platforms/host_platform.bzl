"""Utilities for gating Bazel targets based on the host platform."""

load("@platforms//host:constraints.bzl", "HOST_CONSTRAINTS")

def host_compatible_with(required_constraints):
    """Returns constraints that skip this target on incompatible hosts.

    Args:
        required_constraints: list of constraint value labels that must all
            be present in HOST_CONSTRAINTS for the host to be compatible.

    Returns:
        ["@platforms//:incompatible"] if any required constraint is missing,
        otherwise [].
    """
    for c in required_constraints:
        if c not in HOST_CONSTRAINTS:
            return ["@platforms//:incompatible"]
    return []
