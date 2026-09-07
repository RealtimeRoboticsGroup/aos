# Description:
#   Compatibility shim. The flatbuffer rules split by language -- the codegen
#   core is defs.bzl and the wrappers are cc.bzl, rust.bzl, and go.bzl -- so
#   that a consumer only loads the rulesets for the languages it generates.
#   This file re-exports the old build_defs.bzl surface for out-of-tree code
#   that still loads it.
#
#   Loading this file costs you rules_go and rules_rust, exactly the tax the
#   split exists to avoid. Load the per-language file instead; nothing inside
#   AOS loads this one.
#
#   (flatbuffer_py_library and flatbuffer_ts_library are not here: their old
#   homes were flatbuffers_python.bzl and typescript.bzl, which live on as
#   their own shims next door.)

"""Deprecated compatibility re-exports; load the per-language .bzl instead."""

load(":cc.bzl", _flatbuffer_cc_library = "flatbuffer_cc_library")
load(
    ":defs.bzl",
    _DEFAULT_FLATC_ARGS = "DEFAULT_FLATC_ARGS",
    _DEFAULT_FLATC_GO_ARGS = "DEFAULT_FLATC_GO_ARGS",
    _DEFAULT_FLATC_RUST_ARGS = "DEFAULT_FLATC_RUST_ARGS",
    _DEFAULT_INCLUDE_PATHS = "DEFAULT_INCLUDE_PATHS",
    _FlatbufferLibraryInfo = "FlatbufferLibraryInfo",
    _flatbuffer_library_public = "flatbuffer_library_public",
    _flatc_path = "flatc_path",
)
load(":go.bzl", _flatbuffer_go_library = "flatbuffer_go_library")
load(":rust.bzl", _flatbuffer_rust_library = "flatbuffer_rust_library")

flatc_path = _flatc_path
DEFAULT_INCLUDE_PATHS = _DEFAULT_INCLUDE_PATHS
DEFAULT_FLATC_ARGS = _DEFAULT_FLATC_ARGS
DEFAULT_FLATC_GO_ARGS = _DEFAULT_FLATC_GO_ARGS
DEFAULT_FLATC_RUST_ARGS = _DEFAULT_FLATC_RUST_ARGS
FlatbufferLibraryInfo = _FlatbufferLibraryInfo
flatbuffer_library_public = _flatbuffer_library_public
flatbuffer_cc_library = _flatbuffer_cc_library
flatbuffer_go_library = _flatbuffer_go_library
flatbuffer_rust_library = _flatbuffer_rust_library
