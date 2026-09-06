# Description:
#   Compatibility shim: the python rules moved to python.bzl to match the
#   per-language naming (cc.bzl, rust.bzl, go.bzl, ts.bzl). Out-of-tree BUILD
#   files load the old name -- convert_msg.py used to emit this load line into
#   generated BUILD files that are kept as source.

"""Deprecated compatibility re-exports; load python.bzl instead."""

load(
    ":python.bzl",
    _FLATC_ARGS = "FLATC_ARGS",
    _FLATC_PATH = "FLATC_PATH",
    _FlatbufferPyInfo = "FlatbufferPyInfo",
    _flatbuffer_py_library = "flatbuffer_py_library",
    _flatbuffer_py_srcs = "flatbuffer_py_srcs",
)

FLATC_PATH = _FLATC_PATH
FLATC_ARGS = _FLATC_ARGS
FlatbufferPyInfo = _FlatbufferPyInfo
flatbuffer_py_library = _flatbuffer_py_library
flatbuffer_py_srcs = _flatbuffer_py_srcs
