cc_binary(
    name = "teensy_loader_cli",
    srcs = [
        "teensy_loader_cli.c",
    ],
    copts = [
        "-DUSE_LIBUSB",
    ],
    target_compatible_with = ["@platforms//cpu:x86_64"],
    deps = [
        "@amd64_debian_sysroot//:libusb",
        "@amd64_debian_sysroot//:libusb-1.0",
    ],
)
