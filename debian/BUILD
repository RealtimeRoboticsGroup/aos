load(
    ":apache2.bzl",
    apache2_debs = "files",
)
load(
    ":mingw_compiler.bzl",
    mingw_compiler_debs = "files",
)
load(":packages.bzl", "download_packages", "generate_deb_tarball")
load(
    ":phoenix6.bzl",
    phoenix6_debs = "files",
)
load(
    ":postgresql_amd64.bzl",
    postgresql_amd64_debs = "files",
)
load(
    ":xvfb_amd64.bzl",
    xvfb_amd64_debs = "files",
)

package(default_visibility = ["//visibility:public"])

py_binary(
    name = "download_packages",
    srcs = [
        "download_packages.py",
    ],
    main = "download_packages.py",
    target_compatible_with = ["@platforms//os:linux"],
)

download_packages(
    name = "download_apache2_packages",
    excludes = [
        "libaprutil1-dbd-mysql",
        "libaprutil1-dbd-odbc",
        "libaprutil1-dbd-pgsql",
        "libaprutil1-dbd-freetds",
        "libstdc++6",
        "lsb-base",
        "debconf",
        "libc6-dev",
    ],
    force_includes = [
        "libaprutil1",
    ],
    packages = [
        "apache2",
    ],
)

download_packages(
    name = "download_postgresql_deps",
    excludes = [
        "adduser",
        "debconf",
        "debconf-2.0",
        "libsystemd0",
        "lsb-base",
        "libstdc++6",
        "libc-bin",
        "libc-l10n",
        "netbase",
        "ucf",
        "locales",
        "locales-all",
    ],
    packages = [
        "postgresql",
    ],
    target_compatible_with = ["@platforms//os:linux"],
)

download_packages(
    name = "download_mingw_compiler_deps",
    packages = [
        "g++-mingw-w64-x86-64",
    ],
    target_compatible_with = ["@platforms//os:linux"],
)

generate_deb_tarball(
    name = "phoenix6",
    files = phoenix6_debs,
    target_compatible_with = ["@platforms//os:linux"],
)

generate_deb_tarball(
    name = "apache2",
    files = apache2_debs,
    target_compatible_with = ["@platforms//os:linux"],
)

generate_deb_tarball(
    name = "postgresql_amd64",
    files = postgresql_amd64_debs,
    target_compatible_with = ["@platforms//os:linux"],
)

generate_deb_tarball(
    name = "mingw_compiler",
    files = mingw_compiler_debs,
    target_compatible_with = ["@platforms//os:linux"],
)

download_packages(
    name = "download_xvfb_packages",
    excludes = [
        "libglx-mesa0",
    ],
    packages = [
        "xvfb",
    ],
)

generate_deb_tarball(
    name = "xvfb_amd64",
    files = xvfb_amd64_debs,
    target_compatible_with = ["@platforms//os:linux"],
)

exports_files([
    "curl.BUILD",
    "BUILD.zlib.bazel",
])
