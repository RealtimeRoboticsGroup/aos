load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")
load("//frc/control_loops/drivetrain:drivetrain_config.bzl", "drivetrain_config")
load("//tools/python:defs.bzl", "py_binary", "py_library")

py_binary(
    name = "drivetrain",
    srcs = [
        "drivetrain.py",
    ],
    legacy_create_init = False,
    target_compatible_with = ["//tools/platforms/python:has_support"],
    deps = [
        ":python_init",
        "//frc/control_loops/python:drivetrain",
        "@pip_deps//glog",
        "@pip_deps//python_gflags",
    ],
)

py_binary(
    name = "polydrivetrain",
    srcs = [
        "drivetrain.py",
        "polydrivetrain.py",
    ],
    legacy_create_init = False,
    target_compatible_with = ["//tools/platforms/python:has_support"],
    deps = [
        ":python_init",
        "//frc/control_loops/python:polydrivetrain",
        "@pip_deps//glog",
        "@pip_deps//python_gflags",
    ],
)

genrule(
    name = "genrule_drivetrain",
    outs = [
        "drivetrain_dog_motor_plant.h",
        "drivetrain_dog_motor_plant.cc",
        "drivetrain_dog_motor_plant.json",
        "kalman_drivetrain_motor_plant.h",
        "kalman_drivetrain_motor_plant.cc",
        "kalman_drivetrain_motor_plant.json",
    ],
    cmd = "$(location :drivetrain) $(OUTS)",
    target_compatible_with = ["//tools/platforms/python:has_support"],
    tools = [
        ":drivetrain",
    ],
)

genrule(
    name = "genrule_polydrivetrain",
    outs = [
        "polydrivetrain_dog_motor_plant.h",
        "polydrivetrain_dog_motor_plant.cc",
        "polydrivetrain_dog_motor_plant.json",
        "polydrivetrain_cim_plant.h",
        "polydrivetrain_cim_plant.cc",
        "polydrivetrain_cim_plant.json",
        "hybrid_velocity_drivetrain.h",
        "hybrid_velocity_drivetrain.cc",
        "hybrid_velocity_drivetrain.json",
    ],
    cmd = "$(location :polydrivetrain) $(OUTS)",
    target_compatible_with = ["//tools/platforms/python:has_support"],
    tools = [
        ":polydrivetrain",
    ],
)

cc_library(
    name = "polydrivetrain_plants",
    srcs = [
        "drivetrain_dog_motor_plant.cc",
        "hybrid_velocity_drivetrain.cc",
        "kalman_drivetrain_motor_plant.cc",
        "polydrivetrain_dog_motor_plant.cc",
    ],
    hdrs = [
        "drivetrain_dog_motor_plant.h",
        "hybrid_velocity_drivetrain.h",
        "kalman_drivetrain_motor_plant.h",
        "polydrivetrain_dog_motor_plant.h",
    ],
    target_compatible_with = ["//tools/platforms/python:has_support"],
    visibility = ["//visibility:public"],
    deps = [
        "//frc/control_loops:hybrid_state_feedback_loop",
        "//frc/control_loops:state_feedback_loop",
    ],
)

cc_test(
    name = "drivetrain_json_test",
    srcs = [
        "drivetrain_json_test.cc",
    ],
    data = [
        "drivetrain_dog_motor_plant.json",
        "hybrid_velocity_drivetrain.json",
        "kalman_drivetrain_motor_plant.json",
        "polydrivetrain_cim_plant.json",
        "polydrivetrain_dog_motor_plant.json",
    ],
    deps = [
        ":polydrivetrain_plants",
        "//aos/testing:googletest",
        "//aos/testing:path",
        "//frc/control_loops:hybrid_state_feedback_loop_converters",
        "//frc/control_loops/drivetrain:drivetrain_config_fbs",
    ],
)

py_library(
    name = "python_init",
    srcs = ["__init__.py"],
    target_compatible_with = ["//tools/platforms/python:has_support"],
    visibility = ["//visibility:public"],
)

drivetrain_config(
    name = "drivetrain_config",
    out = "drivetrain_config.json",
)
