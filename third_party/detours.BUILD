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
    includes = ["src"],
    visibility = ["//visibility:public"],
    defines = select({
        "@platforms//os:windows": ["_AMD64_"],
        "//conditions:default": [],
    }),
    linkopts = select({
        "@platforms//os:windows": [
            "-DEFAULTLIB:dbghelp.lib",
        ],
        "//conditions:default": [],
    }),
)
