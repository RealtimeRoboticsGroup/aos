load("//tools/python:defs.bzl", "py_binary", "py_library", "py_test")

package(default_visibility = ["//visibility:public"])

py_library(
    name = "constants",
    srcs = ["constants.py"],
    deps = [
        "@pip_deps//pygobject",
    ],
)

py_library(
    name = "drawing_constants",
    srcs = ["drawing_constants.py"],
    deps = [
        ":color",
        "@pip_deps//numpy",
        "@pip_deps//pygobject",
    ],
)

py_binary(
    name = "haptic_wheel",
    srcs = [
        "haptic_wheel.py",
    ],
    legacy_create_init = False,
    target_compatible_with = ["//tools/platforms/python:has_support"],
    deps = [
        "//frc/control_loops/python:controls",
        "@pip_deps//glog",
        "@pip_deps//matplotlib",
        "@pip_deps//pygobject",
        "@pip_deps//python_gflags",
    ],
)

py_library(
    name = "controls",
    srcs = [
        "cim.py",
        "control_loop.py",
        "controls.py",
        "libcdd.py",
        "polytope.py",
    ],
    data = [
        "//third_party/cddlib:_cddlib.so",
    ],
    target_compatible_with = ["//tools/platforms/python:has_support"],
    deps = [
        ":python_init",
        "@pip_deps//glog",
        "@pip_deps//scipy",
        "@rules_python//python/runfiles",
    ],
)

py_test(
    name = "polytope_test",
    srcs = [
        "polytope_test.py",
    ],
    legacy_create_init = False,
    target_compatible_with = ["//tools/platforms/python:has_support"],
    deps = [
        ":controls",
        ":python_init",
    ],
)

py_binary(
    name = "down_estimator",
    srcs = [
        "down_estimator.py",
    ],
    legacy_create_init = False,
    target_compatible_with = ["//tools/platforms/python:has_support"],
    deps = [
        ":controls",
        ":python_init",
        "@pip_deps//matplotlib",
        "@pip_deps//pygobject",
    ],
)

py_library(
    name = "drivetrain",
    srcs = [
        "drivetrain.py",
    ],
    target_compatible_with = ["//tools/platforms/python:has_support"],
    deps = [
        ":controls",
        ":python_init",
        "@pip_deps//matplotlib",
        "@pip_deps//pygobject",
    ],
)

py_library(
    name = "libspline",
    srcs = [
        "libspline.py",
    ],
    data = [
        "//frc/control_loops/drivetrain:spline.so",
    ],
    target_compatible_with = ["//tools/platforms/python:has_support"],
    deps = [
        ":python_init",
        "@rules_python//python/runfiles",
    ],
)

py_test(
    name = "lib_spline_test",
    srcs = [
        "lib_spline_test.py",
    ],
    target_compatible_with = ["//tools/platforms/python:has_support"],
    deps = [
        ":libspline",
        ":python_init",
        "@pip_deps//numpy",
    ],
)

py_library(
    name = "polydrivetrain",
    srcs = [
        "polydrivetrain.py",
    ],
    target_compatible_with = ["//tools/platforms/python:has_support"],
    deps = [
        ":controls",
        ":drivetrain",
        ":python_init",
        "@pip_deps//glog",
        "@pip_deps//matplotlib",
        "@pip_deps//pygobject",
    ],
)

py_library(
    name = "python_init",
    srcs = ["__init__.py"],
    target_compatible_with = ["//tools/platforms/python:has_support"],
    visibility = ["//visibility:public"],
    deps = ["//frc/control_loops:python_init"],
)

py_library(
    name = "linear_system",
    srcs = ["linear_system.py"],
    target_compatible_with = ["//tools/platforms/python:has_support"],
    visibility = ["//visibility:public"],
    deps = [
        ":controls",
        "//aos/util:py_trapezoid_profile",
        "//frc/control_loops:python_init",
        "@pip_deps//matplotlib",
        "@pip_deps//pygobject",
    ],
)

py_library(
    name = "angular_system",
    srcs = ["angular_system.py"],
    target_compatible_with = ["//tools/platforms/python:has_support"],
    visibility = ["//visibility:public"],
    deps = [
        ":controls",
        "//aos/util:py_trapezoid_profile",
        "//frc/control_loops:python_init",
        "@pip_deps//matplotlib",
        "@pip_deps//pygobject",
    ],
)

py_library(
    name = "angular_system_current",
    srcs = ["angular_system_current.py"],
    target_compatible_with = ["//tools/platforms/python:has_support"],
    visibility = ["//visibility:public"],
    deps = [
        ":controls",
        "//aos/util:py_trapezoid_profile",
        "//frc/control_loops:python_init",
        "@pip_deps//matplotlib",
        "@pip_deps//pygobject",
    ],
)

py_binary(
    name = "spline_graph",
    srcs = [
        "graph.py",
        "multispline.py",
        "path_edit.py",
        "spline_drawing.py",
        "spline_graph.py",
        "spline_writer.py",
    ],
    data = glob([
        "field_images/*.png",
        "field_images/*.svg",
    ]) + ["//third_party/y2023/field:pictures"] + ["//third_party/y2024/field:pictures"],
    legacy_create_init = False,
    target_compatible_with = ["@platforms//cpu:x86_64"],
    visibility = ["//visibility:public"],
    deps = [
        ":basic_window",
        ":color",
        ":constants",
        ":drawing_constants",
        ":libspline",
        ":python_init",
        "@pip_deps//matplotlib",
        "@pip_deps//numpy",
        "@pip_deps//pygobject",
        "@pip_deps//scipy",
    ],
)

py_library(
    name = "basic_window",
    srcs = [
        "basic_window.py",
    ],
    target_compatible_with = ["@platforms//cpu:x86_64"],
    visibility = ["//visibility:public"],
    deps = [
        ":constants",
        ":python_init",
        "@pip_deps//pygobject",
    ],
)

py_library(
    name = "color",
    srcs = [
        "color.py",
    ],
    target_compatible_with = ["//tools/platforms/python:has_support"],
    visibility = ["//visibility:public"],
    deps = [
        ":python_init",
    ],
)

py_binary(
    name = "flywheel",
    srcs = [
        "flywheel.py",
    ],
    legacy_create_init = False,
    target_compatible_with = ["//tools/platforms/python:has_support"],
    deps = [
        ":python_init",
        "//frc/control_loops/python:controls",
        "@pip_deps//matplotlib",
        "@pip_deps//pygobject",
    ],
)

py_binary(
    name = "wrapped_subsystem_test",
    srcs = [
        "wrapped_subsystem_test.py",
    ],
    legacy_create_init = False,
    target_compatible_with = ["//tools/platforms/python:has_support"],
    deps = [
        ":angular_system",
        ":controls",
        ":python_init",
        "@pip_deps//glog",
        "@pip_deps//python_gflags",
    ],
)

py_binary(
    name = "static_zeroing_single_dof_profiled_subsystem_test",
    srcs = [
        "static_zeroing_single_dof_profiled_subsystem_test.py",
    ],
    legacy_create_init = False,
    target_compatible_with = ["//tools/platforms/python:has_support"],
    deps = [
        ":controls",
        ":linear_system",
        ":python_init",
        "@pip_deps//glog",
        "@pip_deps//python_gflags",
    ],
)

py_binary(
    name = "flywheel_controller_test",
    srcs = [
        "flywheel_controller_test.py",
    ],
    legacy_create_init = False,
    target_compatible_with = ["//tools/platforms/python:has_support"],
    deps = [
        ":controls",
        ":flywheel",
        ":python_init",
        "@pip_deps//glog",
        "@pip_deps//python_gflags",
    ],
)
