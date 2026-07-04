load("//:repositories_internal.bzl", "april_tag_test_image_repo", "apriltag_test_bfbs_images_repo", "calibrate_multi_cameras_data_repo", "com_github_nvidia_cccl_repo", "coral_image_thriftycam_2025_repo", "drivetrain_replay_repo", "frc2026_field_map_welded_repo", "intrinsic_calibration_test_images_repo", "orin_capture_24_04_repo", "orin_capture_24_04_side_repo", "orin_image_apriltag_repo", "orin_large_image_apriltag_repo", "superstructure_replay_repo")

def _frc_deps_extension_impl(_ctx):
    frc2026_field_map_welded_repo()

    intrinsic_calibration_test_images_repo()

    orin_image_apriltag_repo()

    orin_large_image_apriltag_repo()

    orin_capture_24_04_repo()

    orin_capture_24_04_side_repo()

    coral_image_thriftycam_2025_repo()

    calibrate_multi_cameras_data_repo()

    apriltag_test_bfbs_images_repo()

    april_tag_test_image_repo()

    drivetrain_replay_repo()

    superstructure_replay_repo()

    com_github_nvidia_cccl_repo()

frc_deps_extension = module_extension(
    implementation = _frc_deps_extension_impl,
)
