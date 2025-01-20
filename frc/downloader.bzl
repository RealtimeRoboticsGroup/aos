load("//frc/downloader:downloader.bzl", "aos_downloader")
load("//tools/build_rules:label.bzl", "expand_label")

def robot_downloader(
        name = "download",
        binaries = [],
        data = [],
        dirs = None,
        target_compatible_with = ["@//tools/platforms/hardware:roborio"],
        target_type = "roborio"):
    """Sets up the standard robot download targets.

    Attrs:
      start_binaries: A list of cc_binary targets to start on the robot.
      dirs: Passed through to aos_downloader.
    """

    aos_downloader(
        name = name,
        srcs = ([
            "//aos:prime_start_binaries",
        ] if target_type == "roborio" else []) + [
            "//aos:prime_binaries",
        ] + binaries + data + ["//frc/raspi/rootfs:chrt.sh"],
        dirs = dirs,
        target_type = target_type,
        target_compatible_with = target_compatible_with,
    )

    aos_downloader(
        name = name + "_stripped",
        srcs = ([
            "//aos:prime_start_binaries_stripped",
        ] if target_type == "roborio" else []) + [
            "//aos:prime_binaries_stripped",
        ] + [expand_label(binary) + ".stripped" for binary in binaries] + data + ["//frc/raspi/rootfs:chrt.sh"],
        dirs = dirs,
        target_type = target_type,
        target_compatible_with = target_compatible_with,
    )
