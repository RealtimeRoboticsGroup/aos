load("@aos//aos/flatbuffers:build_defs.bzl", "flatbuffer_cc_library")
load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("//tools/build_rules:clean_dep.bzl", "clean_dep")

def _static_flatbuffer_gen_impl(ctx):
    """Implementation for generating static flatbuffer headers."""
    inputs = ctx.files.bfbs_files
    schema_files = ctx.attr.base_files
    outputs = ctx.outputs.outs

    for i in range(len(inputs)):
        ctx.actions.run(
            executable = ctx.executable._generate,
            arguments = [
                "--reflection_bfbs",
                inputs[i].path,
                "--output_file",
                outputs[i].path,
                "--base_file_name",
                schema_files[i],
            ],
            inputs = [inputs[i]],
            outputs = [outputs[i]],
            mnemonic = "StaticFlatbufferGen",
            progress_message = "Generating static flatbuffer header for %s" % schema_files[i],
        )
    return [DefaultInfo(files = depset(outputs))]

_static_flatbuffer_gen = rule(
    implementation = _static_flatbuffer_gen_impl,
    attrs = {
        "base_files": attr.string_list(mandatory = True),
        "bfbs_files": attr.label(mandatory = True, allow_files = True),
        "outs": attr.output_list(mandatory = True),
        "_generate": attr.label(
            executable = True,
            cfg = "exec",
            default = Label(clean_dep("//aos/flatbuffers:generate")),
        ),
    },
)

def static_flatbuffer(name, visibility = None, deps = [], srcs = [], **kwargs):
    """Generates the code for the static C++ flatbuffer API for the specified fbs file.

    Generates a cc_library of name name that can be depended on by C++ code and other
    static_flatbuffer rules.

    The cc_library will consist of a single file suffixed with _static.h and prefixed
    with the name of the flatbuffer file itself (i.e., if you have a src of foo.fbs, then
    the resulting header will be foo_static.h).

    Args:
      name: Target name.
      srcs: List of fbs files to codegen.
      visibility: Desired rule visibility.
      deps: List of static_flatbuffer dependencies of this rule.
    """
    fbs_suffix = "_fbs"

    flatbuffer_cc_library(
        name = name + fbs_suffix,
        srcs = srcs,
        deps = [dep + fbs_suffix for dep in deps],
        gen_reflections = True,
        visibility = visibility,
        **kwargs
    )

    cleaned_srcs = []
    for file in srcs:
        if file.startswith(":"):
            cleaned_srcs.append(file[1:])
        elif ":" in file or "@" in file or "//" in file:
            fail("Invalid file (may not be in current package): %s" % file)
        else:
            cleaned_srcs.append(file)

    # Until we make this a proper rule with providers or the such, we just manage headers
    # by having a strong convention where the header will be a function of the fbs name
    # rather than a function of the rule name.
    header_names = [file.removesuffix(".fbs") + "_static.h" for file in cleaned_srcs]
    reflection_out = name + fbs_suffix + "_reflection_out"

    _static_flatbuffer_gen(
        name = name + "_gen",
        bfbs_files = reflection_out,
        outs = header_names,
        base_files = [native.package_name() + "/" + file for file in cleaned_srcs],
    )
    cc_library(
        name = name,
        hdrs = header_names,
        deps = [clean_dep("//aos/flatbuffers:static_table"), clean_dep("//aos/flatbuffers:static_vector"), clean_dep("//aos:macros"), name + fbs_suffix] + deps,
        visibility = visibility,
    )
    native.alias(
        name = name + "_reflection_out",
        actual = name + fbs_suffix + "_reflection_out",
        visibility = visibility,
    )
