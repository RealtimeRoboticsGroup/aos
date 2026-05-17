load("@rules_cc//cc:cc_binary.bzl", "cc_binary")
load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")
load("//third_party/pico-sdk:hex.bzl", "pioasm", "uf2_from_elf")

# TODO(austin): We should be able to either push the linker script into the crosstool, or wrap this in a macro.
cc_binary(
    name = "blink.elf",
    srcs = [
        "blink.c",
        "//third_party/pico-sdk/src/rp2_common/boot_stage2:bs2_default_padded_checksummed.S",
    ],
    additional_linker_inputs = [
        "//third_party/pico-sdk:src/rp2_common/pico_standard_link/memmap_default.ld",
    ],
    linkopts = [
        "-Wl,--script=third_party/pico-sdk/src/rp2_common/pico_standard_link/memmap_default.ld",
    ],
    target_compatible_with = [
        "@platforms//os:none",
        "//tools/platforms/hardware:cortex_m0plus",
    ],
    deps = ["//third_party/pico-sdk"],
)

uf2_from_elf(
    name = "blink",
)

pioasm(
    name = "quadrature_encoder",
    src = "quadrature_encoder.pio",
    out = "quadrature_encoder.pio.h",
)

# TODO(austin): We should be able to either push the linker script into the crosstool, or wrap this in a macro.
cc_binary(
    name = "ADIS16505.elf",
    srcs = [
        "ADIS16505.cc",
        "quadrature_encoder.pio.h",
        "//third_party/pico-sdk/src/rp2_common/boot_stage2:bs2_default_padded_checksummed.S",
    ],
    additional_linker_inputs = [
        "//third_party/pico-sdk:src/rp2_common/pico_standard_link/memmap_default.ld",
    ],
    copts = [
        "-Wno-unused-parameter",
        "-Wno-unused-variable",
        "-Wno-double-promotion",
        "-Wno-unused-function",
    ],
    includes = ["."],
    linkopts = [
        "-Wl,--script=third_party/pico-sdk/src/rp2_common/pico_standard_link/memmap_default.ld",
    ],
    target_compatible_with = [
        "@platforms//os:none",
        "//tools/platforms/hardware:cortex_m0plus",
    ],
    deps = ["//third_party/pico-sdk"],
)

uf2_from_elf(
    name = "ADIS16505",
)

cc_library(
    name = "imu_calibrator",
    srcs = ["imu_calibrator.cc"],
    hdrs = [
        "imu_calibrator.h",
        "imu_calibrator-tmpl.h",
    ],
    deps = [
        "//aos/time",
        "//frc/math:interpolate",
        "@ceres-solver//:ceres",
        "@com_google_absl//absl/flags:flag",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/strings:str_format",
        "@eigen",
    ],
)

cc_library(
    name = "imu_calibrator_solver",
    srcs = [
        "imu_calibrator_solver.cc",
    ],
    hdrs = [
        "imu_calibrator_solver.h",
    ],
    deps = [
        ":imu_calibrator",
        "@ceres-solver//:ceres",
        "@com_google_absl//absl/flags:flag",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
        "@eigen",
    ],
)

cc_test(
    name = "imu_calibrator_test",
    srcs = ["imu_calibrator_test.cc"],
    shard_count = 3,
    deps = [
        ":imu_calibrator",
        ":imu_calibrator_solver",
        "//aos/testing:googletest",
    ],
)
