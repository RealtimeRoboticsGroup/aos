load("//frc/downloader:downloader.bzl", "aos_downloader_dir")
package(default_visibility = ["//visibility:public"])

filegroup(
    name = "trajectories_jsons",
    srcs = glob([
        "trajectories/*.json"
    ]),
)

aos_downloader_dir(
    name = "trajectories",
    srcs = [
        ":trajectories_jsons",
    ],
    dir = "trajectories",
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
)


cc_binary(
    name = "autonomous_main",
    srcs = ["autonomous_main.cc"],
    target_compatible_with = ["//tools/platforms/hardware:roborio"],
    deps = [
        "//aos:init",
        "//aos/events:shm_event_loop",
        "//frc/control_loops/swerve:autonomous_pd_loop",
        "//y2024_bot3/control_loops/superstructure:superstructure_goal_fbs",
    ]
)
