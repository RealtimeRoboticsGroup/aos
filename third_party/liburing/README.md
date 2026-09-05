# liburing Bazel build support

`add-bazel-build-support.patch` is generated from the liburing `2.14.bcr.2`
overlay in the Bazel Central Registry:

    https://bcr.bazel.build/modules/liburing/2.14.bcr.2/overlay/BUILD.bazel
    https://bcr.bazel.build/modules/liburing/2.14.bcr.2/overlay/liburing_configure.bzl

Under bzlmod the registry applies that overlay itself and this patch is unused.
WORKSPACE mode has no way to apply a registry overlay, so `repositories.bzl`
patches the same two files into the `@liburing` archive instead.  Both modes
therefore build identical sources.

To regenerate after a liburing bump, drop the two overlay files into an empty
git repository and `git diff --cached` them against an empty commit, then
update the version in `MODULE.bazel` and `repositories.bzl` to match.

This whole directory goes away with WORKSPACE support.
