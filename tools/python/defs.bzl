load("@rules_python//python:defs.bzl", _py_binary = "py_binary", _py_library = "py_library", _py_test = "py_test")

def py_binary(target_compatible_with = ["@aos//tools/platforms/python:has_support"], **kwargs):
    _py_binary(
        target_compatible_with = target_compatible_with,
        **kwargs
    )

def py_library(target_compatible_with = ["@aos//tools/platforms/python:has_support"], **kwargs):
    _py_library(
        target_compatible_with = target_compatible_with,
        **kwargs
    )

def py_test(target_compatible_with = ["@aos//tools/platforms/python:has_support"], **kwargs):
    _py_test(
        target_compatible_with = select({
            Label("@aos//tools:has_msan"): ["@platforms//:incompatible"],
            Label("//conditions:default"): target_compatible_with,
        }),
        **kwargs
    )
