"""Public build-macro entry point for AOS consumers.

External repositories should load AOS macros from here
(`load("@aos//aos:defs.bzl", "aos_config", "static_flatbuffer", ...)`) rather
than from the implementation files, so AOS is free to move those without
breaking consumers. This also helps define a prefered API for users.
"""

load("//aos:config.bzl", _aos_config = "aos_config")
load("//aos:flatbuffers.bzl", _cc_static_flatbuffer = "cc_static_flatbuffer")
load("//aos/flatbuffers:build_defs.bzl", _flatbuffer_cc_library = "flatbuffer_cc_library")
load("//aos/flatbuffers:flatbuffers_python.bzl", _flatbuffer_py_library = "flatbuffer_py_library")
load("//aos/flatbuffers:generate.bzl", _static_flatbuffer = "static_flatbuffer")

# Flattens a .json AOS config (channels, applications, imports) into the
# .stripped.json/.bfbs pair that aos::configuration::ReadConfig() consumes.
# NOTE: the target name must differ from the src basename.
aos_config = _aos_config

# Generates the C++ static flatbuffer API (<file>_static.h) plus the regular
# generated API, as a single cc_library.
static_flatbuffer = _static_flatbuffer

# Generates Python bindings for a .fbs schema.
flatbuffer_py_library = _flatbuffer_py_library

# The raw generated-API cc_library, without the static API.
flatbuffer_cc_library = _flatbuffer_cc_library

# Embeds a .bfbs schema in a C++ function returning its Span.
cc_static_flatbuffer = _cc_static_flatbuffer
