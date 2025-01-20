load("//aos/flatbuffers:generate.bzl", "static_flatbuffer")

package(default_visibility = ["//visibility:public"])

static_flatbuffer(
    name = "superstructure_goal_fbs",
    srcs = [
        "superstructure_goal.fbs",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "//frc/control_loops:control_loops_fbs",
        "//frc/control_loops:profiled_subsystem_fbs",
    ],
)
