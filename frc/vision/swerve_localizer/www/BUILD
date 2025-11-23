load("@aspect_bazel_lib//lib:copy_to_directory.bzl", "copy_to_directory")
load("//frc/downloader:downloader.bzl", "aos_downloader_dir")
load("//tools/build_rules:js.bzl", "rollup_bundle", "ts_project")

filegroup(
    name = "files",
    srcs = glob([
        "**/*.html",
        "**/*.css",
        "**/*.png",
    ]) + ["2025.png"],
    visibility = ["//visibility:public"],
)

genrule(
    name = "2025_field_png",
    srcs = ["//third_party/y2025/field:pictures"],
    outs = ["2025.png"],
    cmd = "cp third_party/y2025/field/2025.png $@",
)

ts_project(
    name = "field_main",
    srcs = [
        "constants.ts",
        "field_handler.ts",
        "field_main.ts",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "//aos/network:connect_ts_fbs",
        "//aos/network:message_bridge_client_ts_fbs",
        "//aos/network:message_bridge_server_ts_fbs",
        "//aos/network:web_proxy_ts_fbs",
        "//aos/network/www:proxy",
        "//frc/control_loops:control_loops_ts_fbs",
        "//frc/control_loops/drivetrain:drivetrain_status_ts_fbs",
        "//frc/control_loops/drivetrain/localization:localizer_output_ts_fbs",
        "//frc/vision:game_piece_locations_ts_fbs",
        "//frc/vision:target_map_ts_fbs",
        "//frc/vision/swerve_localizer:status_ts_fbs",
        "//frc/vision/swerve_localizer:visualization_ts_fbs",
        "@com_github_google_flatbuffers//ts:flatbuffers_ts",
    ],
)

rollup_bundle(
    name = "field_main_bundle",
    entry_point = "field_main.js",
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//frc/vision:__subpackages__"],
    deps = [
        ":field_main",
    ],
)

copy_to_directory(
    name = "www_directory",
    srcs = [
        ":www_files_filegroup",
    ],
    add_directory_to_runfiles = True,
    replace_prefixes = {"frc/www": ""},
    visibility = ["//visibility:public"],
)

filegroup(
    name = "www_files_filegroup",
    srcs = [
        ":field_main_bundle.min.js",
        ":files",
        "//frc/www:starter_files",
    ],
    visibility = ["//visibility:public"],
)

aos_downloader_dir(
    name = "www_files",
    srcs = [
        ":field_main_bundle.min.js",
        ":files",
        "//frc/www:starter_files",
    ],
    dir = "www",
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
)
