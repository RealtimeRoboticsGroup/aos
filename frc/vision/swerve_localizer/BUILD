cc_library(
    name = "simulated_constants_sender_lib",
    hdrs = ["simulated_constants_sender_lib.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":field_map_constants_sender_lib",
        "//aos/events:simulated_event_loop",
        "//frc/vision:camera_constants_fbs",
        "//frc/vision:camera_constants_list_fbs",
        "//frc/vision:field_map_fbs",
    ],
)

cc_library(
    name = "field_map_constants_sender_lib",
    srcs = ["field_map_constants_sender_lib.cc"],
    hdrs = ["field_map_constants_sender_lib.h"],
    data = [
        "@frc2025_field_map_welded//file:frc2025r2.fmap",
    ],
    deps = [
        "//aos:json_to_flatbuffer",
        "//aos/events:event_loop",
        "//frc/vision:field_map_fbs",
        "//frc/vision:target_map_fbs",
        "@org_tuxfamily_eigen//:eigen",
    ],
)

cc_binary(
    name = "field_map_constants_sender",
    srcs = [
        "field_map_constants_sender.cc",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":field_map_constants_sender_lib",
        "//aos:configuration",
        "//aos:init",
        "//aos:json_to_flatbuffer",
        "//aos/events:shm_event_loop",
        "//frc/vision:field_map_fbs",
    ],
)
