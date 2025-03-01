cc_binary(
    name = "field_map_constants_sender",
    srcs = [
        "field_map_constants_sender.cc",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//aos:configuration",
        "//aos:init",
        "//aos:json_to_flatbuffer",
        "//aos/events:shm_event_loop",
        "//frc/vision:field_map_fbs",
        "//frc/vision:target_map_fbs",
        "@org_tuxfamily_eigen//:eigen",
    ],
)
