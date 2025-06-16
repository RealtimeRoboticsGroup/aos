load("@rules_python//python:pip.bzl", "package_annotation")

ANNOTATIONS = {
    "matplotlib": package_annotation(
        data = ["@amd64_debian_sysroot//:python_sysroot_files"],
        deps = ["@rules_python//python/runfiles"],
    ),
    "pygobject": package_annotation(
        data = ["@amd64_debian_sysroot//:python_sysroot_files"],
        deps = ["@rules_python//python/runfiles"],
    ),
    "python-gflags": package_annotation(
        deps = ["@pip_deps_six//:pkg"],
    ),
}
