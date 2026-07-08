load("//aos:config.bzl", "aos_config")
load("//tools/build_rules:js.bzl", "rollup_bundle", "ts_project")

exports_files(["styles.css"])

filegroup(
    name = "files",
    srcs = glob([
        "**/*.html",
        "**/*.css",
    ]),
    visibility = ["//visibility:public"],
)

ts_project(
    name = "proxy",
    srcs = [
        "config_handler.ts",
        "proxy.ts",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":reflection_ts",
        "//:node_modules/flatbuffers_reflection",
        "//aos:configuration_ts_fbs",
        "//aos/network:connect_ts_fbs",
        "//aos/network:web_proxy_ts_fbs",
        "@aos//aos/flatbuffers/reflection:reflection_ts_fbs",
        "@com_github_google_flatbuffers//ts:flatbuffers_ts",
    ],
)

ts_project(
    name = "main",
    srcs = [
        "main.ts",
        "ping_handler.ts",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":proxy",
        "//aos/testing/ping_pong:ping_ts_fbs",
        "@com_github_google_flatbuffers//ts:flatbuffers_ts",
    ],
)

rollup_bundle(
    name = "main_bundle",
    entry_point = "main.js",
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//aos:__subpackages__"],
    deps = [
        ":main",
    ],
)

ts_project(
    name = "reflection_test_main",
    srcs = [
        "reflection_test_main.ts",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":reflection_ts",
        "//:node_modules/flatbuffers_reflection",
        "//aos:configuration_ts_fbs",
        "//aos:json_to_flatbuffer_fbs_ts",
        "//aos/network/www:proxy",
        "@com_github_google_flatbuffers//ts:flatbuffers_ts",
    ],
)

ts_project(
    name = "reflection_ts",
    srcs = ["reflection.ts"],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "//:node_modules/flatbuffers_reflection",
        "//aos:configuration_ts_fbs",
        "@aos//aos/flatbuffers/reflection:reflection_ts_fbs",
        "@com_github_google_flatbuffers//ts:flatbuffers_ts",
    ],
)

ts_project(
    name = "colors",
    srcs = [
        "colors.ts",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
)

ts_project(
    name = "plotter",
    srcs = [
        "plotter.ts",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [":colors"],
)

ts_project(
    name = "aos_plotter",
    srcs = [
        "aos_plotter.ts",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":plotter",
        ":proxy",
        ":reflection_ts",
        "//aos:configuration_ts_fbs",
        "//aos/network:connect_ts_fbs",
        "//aos/network:web_proxy_ts_fbs",
        "@aos//aos/flatbuffers/reflection:reflection_ts_fbs",
        "@com_github_google_flatbuffers//ts:flatbuffers_ts",
    ],
)

ts_project(
    name = "demo_plot",
    srcs = [
        "demo_plot.ts",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":aos_plotter",
        ":plotter",
        ":proxy",
        ":reflection_ts",
        "//:node_modules/flatbuffers_reflection",
        "//aos:configuration_ts_fbs",
        "//aos/network:connect_ts_fbs",
        "//aos/network:web_proxy_ts_fbs",
        "@com_github_google_flatbuffers//ts:flatbuffers_ts",
    ],
)

aos_config(
    name = "test_config",
    src = "test_config_file.json",
    flatbuffers = [
        "//aos:configuration_fbs",
        "//aos:json_to_flatbuffer_fbs",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "//aos/events:aos_config",
    ],
)

rollup_bundle(
    name = "reflection_test_bundle",
    entry_point = "reflection_test_main.js",
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":reflection_test_main",
    ],
)

sh_binary(
    name = "web_proxy_demo",
    srcs = ["web_proxy_demo.sh"],
    data = [
        ":reflection_test.html",
        ":reflection_test_bundle.min.js",
        ":test_config",
        "//aos/network:web_proxy_main",
    ],
    target_compatible_with = ["@platforms//os:linux"],
)
