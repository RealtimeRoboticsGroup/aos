"""Stubbed out for bzlmod consumers — TS flatbuffers require dev-only deps."""

def flatbuffer_ts_library(name, **kwargs):
    native.filegroup(
        name = name,
        tags = ["manual"],
        visibility = kwargs.get("visibility"),
    )
