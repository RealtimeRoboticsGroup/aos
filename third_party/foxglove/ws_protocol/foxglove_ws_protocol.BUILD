# MIT License
licenses(["notice"])

cc_library(
    name = "com_github_foxglove_ws-protocol",
    srcs = [
        "cpp/foxglove-websocket/src/base64.cpp",
        "cpp/foxglove-websocket/src/parameter.cpp",
        "cpp/foxglove-websocket/src/serialization.cpp",
        "cpp/foxglove-websocket/src/server_factory.cpp",
    ],
    hdrs = [
        "cpp/foxglove-websocket/include/foxglove/websocket/base64.hpp",
        "cpp/foxglove-websocket/include/foxglove/websocket/callback_queue.hpp",
        "cpp/foxglove-websocket/include/foxglove/websocket/common.hpp",
        "cpp/foxglove-websocket/include/foxglove/websocket/parameter.hpp",
        "cpp/foxglove-websocket/include/foxglove/websocket/regex_utils.hpp",
        "cpp/foxglove-websocket/include/foxglove/websocket/serialization.hpp",
        "cpp/foxglove-websocket/include/foxglove/websocket/server_factory.hpp",
        "cpp/foxglove-websocket/include/foxglove/websocket/server_interface.hpp",
        "cpp/foxglove-websocket/include/foxglove/websocket/websocket_client.hpp",
        "cpp/foxglove-websocket/include/foxglove/websocket/websocket_logging.hpp",
        "cpp/foxglove-websocket/include/foxglove/websocket/websocket_notls.hpp",
        "cpp/foxglove-websocket/include/foxglove/websocket/websocket_server.hpp",
        "cpp/foxglove-websocket/include/foxglove/websocket/websocket_tls.hpp",
    ],
    includes = ["cpp/foxglove-websocket/include/"],
    visibility = ["//visibility:public"],
    deps = [
        "@com_github_nlohmann_json//:nlohmann_json",
        "@com_github_zaphoyd_websocketpp",
    ],
)

cc_binary(
    name = "example_server_json",
    srcs = ["cpp/examples/src/example_server_json.cpp"],
    deps = [":com_github_foxglove_ws-protocol"],
)
