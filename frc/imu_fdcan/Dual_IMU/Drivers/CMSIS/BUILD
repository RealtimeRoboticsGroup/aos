cc_library(
    name = "cmsis",
    hdrs = glob([
        "**/*.h",
    ]),
    defines = [
        "STM32G473xx",
        "STM32G4",
        "USE_HAL_DRIVER",
    ],
    includes = [
        "Device/ST/STM32G4xx/Include",
        "Include",
    ],
    visibility = ["//frc/imu_fdcan:__subpackages__"],
    deps = ["//frc/imu_fdcan/Dual_IMU/Core:config"],
)
