# Description:
#   Compatibility shim: the TypeScript rules moved to ts.bzl to match the
#   per-language naming (cc.bzl, rust.bzl, go.bzl, python.bzl). This keeps
#   out-of-tree loads of the old name working.

"""Deprecated compatibility re-exports; load ts.bzl instead."""

load(
    ":ts.bzl",
    _DEFAULT_FLATC_TS_ARGS = "DEFAULT_FLATC_TS_ARGS",
    _flatbuffer_ts_library = "flatbuffer_ts_library",
)

DEFAULT_FLATC_TS_ARGS = _DEFAULT_FLATC_TS_ARGS
flatbuffer_ts_library = _flatbuffer_ts_library
