load("//frc:halide.bzl", "halide_library")

halide_library(
    name = "ToGreyscaleAndDecimateHalide",
    src = "april_generator.cc",
    args = "rows=720 cols=1280",
    function = "decimate_generator",
    visibility = ["//visibility:public"],
)

halide_library(
    name = "ThresholdHalide",
    src = "april_generator.cc",
    args = "rows=360 cols=640",
    function = "threshold_generator",
    visibility = ["//visibility:public"],
)
