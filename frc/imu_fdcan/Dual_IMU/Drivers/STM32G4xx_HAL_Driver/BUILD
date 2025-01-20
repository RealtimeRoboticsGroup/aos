cc_library(
    name = "hal_driver",
    srcs = glob([
        "Src/*.c",
    ]),
    hdrs = glob([
        "Inc/**/*.h",
    ]),
    includes = ["Inc"],
    visibility = ["//frc/imu_fdcan:__subpackages__"],
    deps = [
        "//frc/imu_fdcan/Dual_IMU/Core:config",
        "//frc/imu_fdcan/Dual_IMU/Drivers/CMSIS:cmsis",
    ],
)
