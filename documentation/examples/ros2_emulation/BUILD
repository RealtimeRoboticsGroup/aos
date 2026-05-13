load("@aos//aos/flatbuffers:flatbuffers_python.bzl", "flatbuffer_py_library")
load("@aos//aos/flatbuffers:generate.bzl", "static_flatbuffer")
load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_python//python:defs.bzl", "py_library")

py_library(
    name = "aos_ros2_node",
    srcs = ["aos_ros2_node.py"],
    visibility = ["//visibility:public"],
)

flatbuffer_py_library(
    name = "ros2_header_fbs_py",
    src = "ros2_header.fbs",
    visibility = ["//visibility:public"],
)

static_flatbuffer(
    name = "ros2_header_fbs",
    srcs = ["ros2_header.fbs"],
    visibility = ["//visibility:public"],
)

flatbuffer_py_library(
    name = "ros2_geometry_vector3_fbs_py",
    src = "ros2_geometry_vector3.fbs",
    visibility = ["//visibility:public"],
)

static_flatbuffer(
    name = "ros2_geometry_vector3_fbs",
    srcs = ["ros2_geometry_vector3.fbs"],
    visibility = ["//visibility:public"],
)

flatbuffer_py_library(
    name = "ros2_geometry_point_fbs_py",
    src = "ros2_geometry_point.fbs",
    visibility = ["//visibility:public"],
)

static_flatbuffer(
    name = "ros2_geometry_point_fbs",
    srcs = ["ros2_geometry_point.fbs"],
    visibility = ["//visibility:public"],
)

flatbuffer_py_library(
    name = "ros2_geometry_quaternion_fbs_py",
    src = "ros2_geometry_quaternion.fbs",
    visibility = ["//visibility:public"],
)

static_flatbuffer(
    name = "ros2_geometry_quaternion_fbs",
    srcs = ["ros2_geometry_quaternion.fbs"],
    visibility = ["//visibility:public"],
)

flatbuffer_py_library(
    name = "ros2_geometry_pose_fbs_py",
    src = "ros2_geometry_pose.fbs",
    visibility = ["//visibility:public"],
    deps = [
        ":ros2_geometry_point_fbs_py_srcs",
        ":ros2_geometry_quaternion_fbs_py_srcs",
    ],
)

static_flatbuffer(
    name = "ros2_geometry_pose_fbs",
    srcs = ["ros2_geometry_pose.fbs"],
    visibility = ["//visibility:public"],
    deps = [
        ":ros2_geometry_point_fbs",
        ":ros2_geometry_quaternion_fbs",
    ],
)

flatbuffer_py_library(
    name = "ros2_sensor_joy_fbs_py",
    src = "ros2_sensor_joy.fbs",
    visibility = ["//visibility:public"],
    deps = [
        ":ros2_header_fbs_py_srcs",
    ],
)

static_flatbuffer(
    name = "ros2_sensor_joy_fbs",
    srcs = ["ros2_sensor_joy.fbs"],
    visibility = ["//visibility:public"],
    deps = [
        ":ros2_header_fbs",
    ],
)

flatbuffer_py_library(
    name = "ros2_string_fbs_py",
    src = "ros2_string.fbs",
    visibility = ["//visibility:public"],
)

static_flatbuffer(
    name = "ros2_string_fbs",
    srcs = ["ros2_string.fbs"],
    visibility = ["//visibility:public"],
)

flatbuffer_py_library(
    name = "ros2_int32_fbs_py",
    src = "ros2_int32.fbs",
    visibility = ["//visibility:public"],
)

static_flatbuffer(
    name = "ros2_int32_fbs",
    srcs = ["ros2_int32.fbs"],
    visibility = ["//visibility:public"],
)

flatbuffer_py_library(
    name = "ros2_u_int64_fbs_py",
    src = "ros2_u_int64.fbs",
    visibility = ["//visibility:public"],
)

static_flatbuffer(
    name = "ros2_u_int64_fbs",
    srcs = ["ros2_u_int64.fbs"],
    visibility = ["//visibility:public"],
)

flatbuffer_py_library(
    name = "ros2_float32_fbs_py",
    src = "ros2_float32.fbs",
    visibility = ["//visibility:public"],
)

static_flatbuffer(
    name = "ros2_float32_fbs",
    srcs = ["ros2_float32.fbs"],
    visibility = ["//visibility:public"],
)

flatbuffer_py_library(
    name = "ros2_bool_fbs_py",
    src = "ros2_bool.fbs",
    visibility = ["//visibility:public"],
)

static_flatbuffer(
    name = "ros2_bool_fbs",
    srcs = ["ros2_bool.fbs"],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "aos_ros2_node_cc",
    hdrs = ["aos_ros2_node.h"],
    visibility = ["//visibility:public"],
    deps = [
        "//aos/events:event_loop",
        "@com_google_absl//absl/strings",
    ],
)
