load("@rules_license//rules:license.bzl", "license")

package(
    default_applicable_licenses = [":license"],
)

license(
    name = "license",
    package_name = "snappy",
    license_kinds = ["@rules_license//licenses/spdx:BSD-3-Clause"],
    package_url = "https://github.com/google/snappy",
    package_version = "fe18b46322727fe4bc6939326ba6bde1e0858b61",
)

genrule(
    name = "make_stubs_public",
    outs = ["snappy-stubs-public.h"],
    cmd = "echo \"#include <cstddef>\n#include <sys/uio.h>\n\" > $@",
)

cc_library(
    name = "snappy",
    srcs = [
        "snappy.cc",
        "snappy-internal.h",
        "snappy-sinksource.cc",
        "snappy-sinksource.h",
        "snappy-stubs-internal.cc",
        "snappy-stubs-internal.h",
    ],
    hdrs = [
        "snappy.h",
        "snappy-stubs-public.h",
    ],
    copts = ["-Wno-sign-compare"],
    visibility = ["//visibility:public"],
)
