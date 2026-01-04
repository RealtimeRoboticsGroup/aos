load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive", "http_file")



def com_github_foxglove_mcap_mcap_repo():
    http_file(
        name = "com_github_foxglove_mcap_mcap",
        executable = True,
        sha256 = "e87895e9af36db629ad01c554258ec03d07b604bc61a0a421449c85223357c71",
        urls = ["https://github.com/foxglove/mcap/releases/download/releases%2Fmcap-cli%2Fv0.0.51/mcap-linux-amd64"],
    )

def frc2025_field_map_welded_repo():
    http_file(
        name = "frc2025_field_map_welded",
        downloaded_file_path = "frc2025r2.fmap",
        sha256 = "20b7621bf988a6e378a252576d43d2bfbd17d4f38ea2cbb2e7f2cfc82a17732a",
        urls = ["https://downloads.limelightvision.io/models/frc2025r2.fmap"],
    )

def intrinsic_calibration_test_images_repo():
    http_archive(
        name = "intrinsic_calibration_test_images",
        build_file_content = """
filegroup(
    name = "intrinsic_calibration_test_images",
    srcs = glob(["**"]),
    visibility = ["//visibility:public"],
)""",
        sha256 = "0359e5c19117835c6ec336233a3bbfe2b273797afe9460bf224496802b8f4055",
        url = "https://realtimeroboticsgroup.org/build-dependencies/intrinsic_calibration_test_images.tar.gz",
    )

def orin_image_apriltag_repo():
    http_file(
        name = "orin_image_apriltag",
        downloaded_file_path = "orin_image_apriltag.bfbs",
        sha256 = "c86604fd0b1301b301e299b1bba2573af8c586413934a386a2bd28fd9b037b84",
        url = "https://realtimeroboticsgroup.org/build-dependencies/orin_image_apriltag.bfbs",
    )

def orin_large_image_apriltag_repo():
    http_file(
        name = "orin_large_image_apriltag",
        downloaded_file_path = "orin_large_gs_apriltag.bfbs",
        sha256 = "d933adac0d6c205c574791060be73701ead05977ff5dd9f6f4eadb45817c3ccb",
        url = "https://realtimeroboticsgroup.org/build-dependencies/orin_large_gs_apriltag.bfbs",
    )

def orin_capture_24_04_repo():
    http_file(
        name = "orin_capture_24_04",
        downloaded_file_path = "orin_capture_24_04.bfbs",
        sha256 = "719edb1d1394c13c1b55d02cf35c277e1d4c2111f4eb4220b28addc08634488a",
        url = "https://realtimeroboticsgroup.org/build-dependencies/orin-capture-24-04-2024.02.14.bfbs",
    )

def orin_capture_24_04_side_repo():
    http_file(
        name = "orin_capture_24_04_side",
        downloaded_file_path = "orin_capture_24_04_side.bfbs",
        sha256 = "4747cc98f8794d6570cb12a3171d7984e358581914a28b43fb6bb8b9bd7a10ac",
        url = "https://realtimeroboticsgroup.org/build-dependencies/orin-capture-24-04-side-2024.02.17.bfbs",
    )

def coral_image_thriftycam_2025_repo():
    http_file(
        name = "coral_image_thriftycam_2025",
        downloaded_file_path = "image.bfbs",
        sha256 = "b746bda7db8a6233a74c59c35f3c9d5e343cd9f9c580c897013e8dff7c492eed",
        urls = ["https://realtimeroboticsgroup.org/build-dependencies/coral_image_thriftycam_2025.bfbs"],
    )

def calibrate_multi_cameras_data_repo():
    http_archive(
        name = "calibrate_multi_cameras_data",
        build_file_content = """
filegroup(
    name = "calibrate_multi_cameras_data",
    srcs = glob(["**"]),
    visibility = ["//visibility:public"],
)""",
        sha256 = "b106b3b975d3cf3ad3fcd5e4be7409f6095e1d531346a90c4ad6bdb7da1d08a5",
        url = "https://realtimeroboticsgroup.org/build-dependencies/2023_calibrate_multi_cameras_data.tar.gz",
    )

def apriltag_test_bfbs_images_repo():
    http_archive(
        name = "apriltag_test_bfbs_images",
        build_file_content = """
filegroup(
    name = "apriltag_test_bfbs_images",
    srcs = glob(["**"]),
    visibility = ["//visibility:public"],
)""",
        sha256 = "2356b9d0b3be59d01e837bfbbee21de55b16232d5e00c66701c20b64ff3272e3",
        url = "https://realtimeroboticsgroup.org/build-dependencies/2023_arducam_apriltag_test_images.tar.gz",
    )

def april_tag_test_image_repo():
    http_archive(
        name = "april_tag_test_image",
        build_file_content = """
filegroup(
    name = "april_tag_test_image",
    srcs = ["test.bfbs", "expected.jpeg", "expected.png"],
    visibility = ["//visibility:public"],
)""",
        sha256 = "5312c79b19e9883b3cebd9d65b4438a2bf05b41da0bcd8c35e19d22c3b2e1859",
        urls = ["https://realtimeroboticsgroup.org/build-dependencies/test_image_frc971.vision.CameraImage_2023.01.28.tar.gz"],
    )

def drivetrain_replay_repo():
    http_archive(
        name = "drivetrain_replay",
        build_file_content = """
filegroup(
    name = "drivetrain_replay",
    srcs = glob(["**/*.bfbs"]),
    visibility = ["//visibility:public"],
)
    """,
        sha256 = "115dcd2fe005cb9cad3325707aa7f4466390c43a08555edf331c06c108bdf692",
        url = "https://realtimeroboticsgroup.org/build-dependencies/2021-03-20_drivetrain_spin_wheels.tar.gz",
    )

def superstructure_replay_repo():
    http_archive(
        name = "superstructure_replay",
        build_file_content = """
filegroup(
    name = "superstructure_replay",
    srcs = glob(["**/*.bfbs"]),
    visibility = ["//visibility:public"],
)
    """,
        sha256 = "2b9a3ecc83f2aba89a1909ae38fe51e6718a5b4d0e7c131846dfb2845df9cd19",
        url = "https://realtimeroboticsgroup.org/build-dependencies/2021-10-03_superstructure_shoot_balls.tar.gz",
    )

def com_github_nvidia_cccl_repo():
    http_archive(
        name = "com_github_nvidia_cccl",
        build_file = "@aos//third_party/cccl:cccl.BUILD",
        sha256 = "38160c628a9e32b7cd55553f299768f72b24074cc9c1a993ba40a177877b3421",
        strip_prefix = "cccl-931dc6793482c61edbc97b7a19256874fd264313",
        url = "https://github.com/NVIDIA/cccl/archive/931dc6793482c61edbc97b7a19256874fd264313.zip",
    )

def arm_frc_linux_gnueabi_repo_repo():
    http_archive(
        name = "arm_frc_linux_gnueabi_repo",
        build_file = "@aos//:registry/modules/arm_frc_linux_gnueabi_repo/2025.2.0/overlay/BUILD.bazel",
        patches = ["@aos//:registry/modules/arm_frc_linux_gnueabi_repo/2025.2.0/patches/fts.patch"],
        sha256 = "e1aea36b35c48d81e146a12a4b7428af051e525fac18c85a53c7be98339cce9f",
        strip_prefix = "roborio-academic",
        url = "https://github.com/wpilibsuite/opensdk/releases/download/v2025-2/cortexa9_vfpv3-roborio-academic-2025-x86_64-linux-gnu-Toolchain-12.1.0.tgz",
    )

def gcc_arm_none_eabi_repo():
    http_archive(
        name = "gcc_arm_none_eabi",
        build_file = "@aos//:compilers/gcc_arm_none_eabi.BUILD",
        sha256 = "6cd1bbc1d9ae57312bcd169ae283153a9572bd6a8e4eeae2fedfbc33b115fdbb",
        strip_prefix = "arm-gnu-toolchain-13.2.Rel1-x86_64-arm-none-eabi",
        url = "https://developer.arm.com/-/media/Files/downloads/gnu/13.2.rel1/binrel/arm-gnu-toolchain-13.2.rel1-x86_64-arm-none-eabi.tar.xz",
    )
