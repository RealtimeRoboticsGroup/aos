load("@rules_cc//cc:cc_library.bzl", "cc_library")

# 3 Clause BSD
licenses(["notice"])

cc_library(
    name = "websocketpp",
    hdrs = glob(["websocketpp/**/*.hpp"]),
    defines = [
        "_WEBSOCKETPP_CPP11_STL_",
        "ASIO_STANDALONE",
    ],
    includes = ["."],
    visibility = ["//visibility:public"],
    deps = [
        "@asio",
        "@boringssl//:ssl",
        "@zlib",
    ],
)
