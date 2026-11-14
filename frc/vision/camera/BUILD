load("@rules_cc//cc:cc_binary.bzl", "cc_binary")
load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")

cc_binary(
    name = "image_logger",
    srcs = [
        "image_logger.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "//aos:configuration",
        "//aos:init",
        "//aos/events:shm_event_loop",
        "//aos/events/logging:log_writer",
        "//aos/logging:log_namer",
        "//aos/util:filesystem_fbs",
        "//frc/input:joystick_state_fbs",
        "//frc/vision:vision_fbs",
    ],
)

cc_library(
    name = "v4l2_reader",
    srcs = [
        "v4l2_reader.cc",
    ],
    hdrs = [
        "v4l2_reader.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "//aos/events:aio",
        "//aos/events:event_loop",
        "//aos/scoped:scoped_fd",
        "//aos/util:threaded_consumer",
        "//frc/vision:camera_constants_fbs",
        "//frc/vision:vision_fbs",
        "@com_google_absl//absl/base",
    ],
)

cc_library(
    name = "cuda_camera_image_callback",
    srcs = ["cuda_camera_image_callback.cc"],
    hdrs = ["cuda_camera_image_callback.h"],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "//aos/events:shm_event_loop",
        "//frc/orin:cuda",
        "//frc/vision/util:charuco_lib",
    ],
)

cc_library(
    name = "media_device",
    srcs = [
        "media_device.cc",
    ],
    hdrs = ["media_device.h"],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "//aos/scoped:scoped_fd",
        "//aos/util:file",
        "@com_google_absl//absl/log:absl_check",
        "@com_google_absl//absl/log:absl_log",
        "@com_google_absl//absl/strings",
    ],
)

cc_binary(
    name = "turbojpeg_decoder",
    srcs = ["turbojpeg_decoder.cc"],
    visibility = ["//visibility:public"],
    deps = [
        "//aos:configuration",
        "//aos:init",
        "//aos/containers:inlined_vector",
        "//aos/events:shm_event_loop",
        "//frc/vision:turbojpeg_decoder_status_fbs",
        "//frc/vision:vision_fbs",
        "//third_party:libturbojpeg",
    ],
)

# Compares 8-bit greyscale images, e.g. a hardware decode with turbojpeg's;
# see gray_image_diff.h.
cc_library(
    name = "gray_image_diff",
    srcs = ["gray_image_diff.cc"],
    hdrs = ["gray_image_diff.h"],
    deps = [
        "@com_google_absl//absl/log:absl_check",
        "@com_google_absl//absl/types:span",
    ],
)

cc_test(
    name = "gray_image_diff_test",
    srcs = ["gray_image_diff_test.cc"],
    deps = [
        ":gray_image_diff",
        "//aos/testing:googletest",
    ],
)

# Thin wrapper around the Jetson NVJPG hardware JPEG decoder; see
# nvjpeg_decoder_lib.h.
cc_library(
    name = "nvjpeg_decoder_lib",
    srcs = ["nvjpeg_decoder_lib.cc"],
    hdrs = ["nvjpeg_decoder_lib.h"],
    # Selects the hardware-extended jpeglib struct layout matching
    # libnvjpeg.so's ABI; must only be set for TUs including the sysroot's
    # libjpeg-8b headers.
    local_defines = ["TEGRA_ACCELERATE"],
    target_compatible_with = [
        "@platforms//cpu:arm64",
        "@platforms//os:linux",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "@arm64_debian_sysroot//:nvbufsurface",
        "@arm64_debian_sysroot//:nvjpeg",
        "@com_google_absl//absl/log:absl_check",
        "@com_google_absl//absl/log:absl_log",
    ],
)

# The NVJPG decoder service, shared by nvjpeg_decoder and
# nvjpeg_decoder_verify; see nvjpeg_decoder_service.h.
cc_library(
    name = "nvjpeg_decoder_service",
    srcs = ["nvjpeg_decoder_service.cc"],
    hdrs = ["nvjpeg_decoder_service.h"],
    target_compatible_with = [
        "@platforms//cpu:arm64",
        "@platforms//os:linux",
    ],
    deps = [
        ":nvjpeg_decoder_lib",
        "//aos:realtime",
        "//aos/containers:inlined_vector",
        "//aos/events:event_loop",
        "//frc/vision:turbojpeg_decoder_status_fbs",
        "//frc/vision:vision_fbs",
        "@com_google_absl//absl/flags:flag",
        "@com_google_absl//absl/log:absl_check",
        "@com_google_absl//absl/log:absl_log",
        "@com_google_absl//absl/strings",
    ],
)

# Hardware (NVJPG engine) counterpart to turbojpeg_decoder with the identical
# channel/flag/status interface -- swap decoders by changing executable_name
# in the config.  arm64-only; x86 simulation keeps using turbojpeg_decoder.
cc_binary(
    name = "nvjpeg_decoder",
    srcs = ["nvjpeg_decoder.cc"],
    target_compatible_with = [
        "@platforms//cpu:arm64",
        "@platforms//os:linux",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":nvjpeg_decoder_service",
        "//aos:configuration",
        "//aos:init",
        "//aos/events:shm_event_loop",
        "@com_google_absl//absl/flags:flag",
    ],
)

# nvjpeg_decoder that also decodes every frame with turbojpeg and dies if the
# hardware output differs.  Swap it in for nvjpeg_decoder (executable_name in
# the config) to check libnvjpeg on a bench or robot; it costs a CPU decode
# per frame, so it is not for matches.
cc_binary(
    name = "nvjpeg_decoder_verify",
    srcs = ["nvjpeg_decoder_verify.cc"],
    target_compatible_with = [
        "@platforms//cpu:arm64",
        "@platforms//os:linux",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":gray_image_diff",
        ":nvjpeg_decoder_service",
        "//aos:configuration",
        "//aos:init",
        "//aos:realtime",
        "//aos/events:shm_event_loop",
        "//third_party:libturbojpeg",
        "@com_google_absl//absl/flags:flag",
        "@com_google_absl//absl/log:absl_check",
        "@com_google_absl//absl/log:absl_log",
    ],
)

# Probe/benchmark for libnvjpeg on the Orin: reports whether the NVJPG
# engine is engaged and the per-frame decode time.  scp to the target and
# run by hand; independent of nvjpeg_decoder_lib's fail-loud policy.
cc_binary(
    name = "nvjpeg_bench",
    srcs = ["nvjpeg_bench.cc"],
    local_defines = ["TEGRA_ACCELERATE"],
    target_compatible_with = [
        "@platforms//cpu:arm64",
        "@platforms//os:linux",
    ],
    deps = [
        "@arm64_debian_sysroot//:nvjpeg",
        "@com_google_absl//absl/log:absl_log",
        "@com_google_absl//absl/log:globals",
        "@com_google_absl//absl/log:initialize",
        "@com_google_absl//absl/strings:str_format",
    ],
)

# Hardware test: decodes DIFFERENT JPEGs through one NvJpegDecoderLib
# instance and checks every output against turbojpeg, to expose stale,
# delayed or wrong-buffer output; see nvjpeg_seq_test.cc.  Needs the NVJPG engine, so CI (x86) skips
# it -- build with --config=arm64 and run it on an Orin.
cc_test(
    name = "nvjpeg_seq_test",
    srcs = ["nvjpeg_seq_test.cc"],
    data = [
        "testdata/camera_a.jpg",
        "testdata/camera_b.jpg",
        "testdata/camera_c.jpg",
        "testdata/seq_040.jpg",
        "testdata/seq_080.jpg",
        "testdata/seq_120.jpg",
        "testdata/seq_200.jpg",
    ],
    target_compatible_with = [
        "@platforms//cpu:arm64",
        "@platforms//os:linux",
    ],
    deps = [
        ":gray_image_diff",
        ":nvjpeg_decoder_lib",
        "//aos/testing:googletest",
        "//aos/testing:path",
        "//third_party:libturbojpeg",
        "@com_google_absl//absl/log:absl_check",
        "@com_google_absl//absl/log:absl_log",
    ],
)

# On-bus latency test for a running decoder service: publishes real camera
# JPEGs and times how long until each one's pixels come back on /gray; see
# decoder_latency_test.cc.  scp to the Orin and run by hand.
cc_binary(
    name = "decoder_latency_test",
    srcs = ["decoder_latency_test.cc"],
    target_compatible_with = [
        "@platforms//cpu:arm64",
        "@platforms//os:linux",
    ],
    deps = [
        ":gray_image_diff",
        "//aos:configuration",
        "//aos:init",
        "//aos/events:shm_event_loop",
        "//frc/vision:vision_fbs",
        "//third_party:libturbojpeg",
        "@com_google_absl//absl/log:absl_check",
        "@com_google_absl//absl/log:absl_log",
        "@com_google_absl//absl/strings:str_format",
    ],
)

# Standalone benchmark for the turbojpeg CPU decode path, mirroring
# turbojpeg_decoder's per-frame calls.  scp to a target and run by hand on a
# .jpg to compare decoder latencies (counterpart to :nvjpeg_bench).
cc_binary(
    name = "turbojpeg_bench",
    srcs = ["turbojpeg_bench.cc"],
    deps = [
        "//third_party:libturbojpeg",
        "@com_google_absl//absl/log:absl_log",
        "@com_google_absl//absl/log:globals",
        "@com_google_absl//absl/log:initialize",
        "@com_google_absl//absl/strings:str_format",
    ],
)

cc_binary(
    name = "camera_reader",
    srcs = [
        "camera_reader_main.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":v4l2_reader",
        "//aos:init",
        "//aos/events:shm_event_loop",
        "//frc/constants:constants_sender_lib",
    ],
)
