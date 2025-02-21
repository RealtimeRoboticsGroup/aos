py_binary(
    name = "downloader",
    srcs = [
        "downloader.py",
    ],
    data = [
        "@amd64_debian_sysroot//:rsync-filegroup",
        "@amd64_debian_sysroot//:ssh-filegroup",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
)
