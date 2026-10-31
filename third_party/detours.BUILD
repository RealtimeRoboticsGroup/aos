load("@rules_cc//cc:cc_library.bzl", "cc_library")

# The Windows SDK headers and detours.h both pick an architecture from the
# _AMD64_ / _ARM64_ defines below.
config_setting(
    name = "windows_arm64",
    constraint_values = [
        "@platforms//cpu:arm64",
        "@platforms//os:windows",
    ],
)

config_setting(
    name = "windows_x86_64",
    constraint_values = [
        "@platforms//cpu:x86_64",
        "@platforms//os:windows",
    ],
)

cc_library(
    name = "detours",
    srcs = [
        "src/creatwth.cpp",
        "src/detours.cpp",
        "src/disasm.cpp",
        "src/image.cpp",
        "src/modules.cpp",
    ],
    hdrs = [
        "src/detours.h",
    ],
    # creatwth.cpp #includes this file directly.
    textual_hdrs = ["src/uimports.cpp"],
    defines = select({
        ":windows_arm64": ["_ARM64_"],
        ":windows_x86_64": ["_AMD64_"],
        "//conditions:default": [],
    }),
    includes = ["src"],
    linkopts = select({
        "@platforms//os:windows": [
            "-DEFAULTLIB:dbghelp.lib",
        ],
        "//conditions:default": [],
    }),
    visibility = ["//visibility:public"],
)
