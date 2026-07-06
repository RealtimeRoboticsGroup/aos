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

def _expand_template_impl(ctx):
    ctx.actions.expand_template(
        template = ctx.file.template,
        output = ctx.outputs.out,
        substitutions = ctx.attr.substitutions,
        is_executable = True,
    )
    return [DefaultInfo(
        files = depset([ctx.outputs.out]),
        executable = ctx.outputs.out,
    )]

expand_template = rule(
    implementation = _expand_template_impl,
    attrs = {
        "out": attr.output(
            mandatory = True,
        ),
        "substitutions": attr.string_dict(
            mandatory = True,
        ),
        "template": attr.label(
            mandatory = True,
            allow_single_file = True,
        ),
    },
    executable = True,
)
