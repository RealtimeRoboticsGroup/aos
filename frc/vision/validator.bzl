load("@aspect_bazel_lib//lib:run_binary.bzl", "run_binary")

# Validates constants.json file and outputs a formatted version.
# TODO(austin): Make this generic
def camera_constants_json(name, src, out):
    run_binary(
        name = name,
        tool = "//frc/vision:camera_constants_formatter",
        srcs = [src],
        outs = [out],
        args = ["$(location %s)" % (src)] + ["$(location %s)" % (out)],
        visibility = ["//visibility:public"],
    )
