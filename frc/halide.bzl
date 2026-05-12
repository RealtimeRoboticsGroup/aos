load("@rules_cc//cc:cc_library.bzl", "cc_library")

def _halide_target_select():
    """Returns the Halide target string based on platform."""
    return select({
        "//tools:arm64_linux": "target=arm-64-linux ",
        "//tools:arm64_macos": "target=arm-64-osx ",
        "@platforms//cpu:x86_64": "target=host ",
        "//conditions:default": "",
    })

def _halide_compatible_with():
    """Returns target_compatible_with based on platform."""
    return select({
        "//tools:arm64_linux": [],
        "//tools:arm64_macos": [],
        "@platforms//cpu:x86_64": [],
        "//conditions:default": ["@platforms//:incompatible"],
    })

def halide_library(name, src, function, args, visibility = None):
    native.genrule(
        name = name + "_build_generator",
        outs = [
            name + "_generator",
        ],
        srcs = [
            src,
        ],
        cmd = "$(location //frc:halide_generator_compile_script) $(OUTS) $(location " + src + ")",
        tools = [
            "//frc:halide_generator_compile_script",
        ],
    )
    native.genrule(
        name = "generate_" + name,
        srcs = [
            ":" + name + "_generator",
        ],
        outs = [
            name + ".h",
            name + ".o",
            name + ".stmt.html",
        ],
        # TODO(austin): Upgrade halide...
        cmd = "$(location :" + name + "_generator) -g '" + function + "' -o $(RULEDIR) -f " + name + " -e 'o,h,html' " + _halide_target_select() + args,
        target_compatible_with = _halide_compatible_with(),
    )
    cc_library(
        name = name,
        srcs = [name + ".o"],
        hdrs = [name + ".h"],
        visibility = visibility,
        deps = [
            "//third_party:halide_runtime",
        ],
    )
