load("@aos//aos/flatbuffers:build_defs.bzl", "flatbuffer_cc_library")
load("@bazel_skylib//lib:paths.bzl", "paths")
load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("//tools/build_rules:clean_dep.bzl", "clean_dep")

StaticFlatbufferReflectionInfo = provider(
    doc = "Reflection metadata for a static_flatbuffer target. Exposes one " +
          "(.fbs, .bfbs) pair per source so downstream codegen tools can walk " +
          "the static_flatbuffer dependency graph via an aspect (propagated " +
          "over the rule's `static_flatbuffer_deps` attribute) and access " +
          "each schema's reflection info.",
    fields = {
        "schemas": "list[struct(bfbs: File, fbs_name: string)]: one entry per " +
                   ".fbs/.bfbs pair owned by this target.",
    },
)

def _static_flatbuffer_impl(ctx):
    # Pair each declared .fbs source with its sibling .bfbs by basename stem
    # (the underlying flatbuffer_cc_library emits `<stem>.bfbs` per `<stem>.fbs`).
    bfbs_by_stem = {
        paths.replace_extension(bfbs.basename, ""): bfbs
        for bfbs in ctx.files.bfbs
    }
    schemas = []
    for fbs_name in ctx.attr.fbs_names:
        stem = paths.replace_extension(paths.basename(fbs_name), "")
        if stem not in bfbs_by_stem:
            fail("No .bfbs file found for '%s' in %s; have %s" %
                 (fbs_name, ctx.attr.bfbs.label, sorted(bfbs_by_stem.keys())))
        schemas.append(struct(bfbs = bfbs_by_stem[stem], fbs_name = fbs_name))

    # Re-export the wrapped cc_library so this target is usable wherever a
    # cc_library is expected (downstream cc_library deps, cc_binary, etc.).
    return [
        ctx.attr.cc_lib[CcInfo],
        ctx.attr.cc_lib[DefaultInfo],
        StaticFlatbufferReflectionInfo(schemas = schemas),
    ]

_static_flatbuffer = rule(
    implementation = _static_flatbuffer_impl,
    attrs = {
        "bfbs": attr.label(
            mandatory = True,
            allow_files = [".bfbs"],
            doc = "Target producing the .bfbs reflection files (one per source).",
        ),
        "cc_lib": attr.label(
            mandatory = True,
            providers = [CcInfo],
            doc = "The underlying cc_library whose CcInfo is re-exported.",
        ),
        "fbs_names": attr.string_list(
            mandatory = True,
            doc = "Source .fbs filenames (parallel to the .bfbs files exposed " +
                  "by `bfbs`, matched by basename stem).",
        ),
        "static_flatbuffer_deps": attr.label_list(
            default = [],
            doc = "Direct user-declared deps, mirrored here for aspect " +
                  "propagation. Kept separate from the wrapped cc_library's " +
                  "`deps` so aspects only traverse the static_flatbuffer dep " +
                  "graph and don't leak into unrelated cc_library transitives.",
        ),
    },
    provides = [CcInfo, StaticFlatbufferReflectionInfo],
)

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

def static_flatbuffer(
        name,
        visibility = None,
        deps = [],
        srcs = [],
        target_compatible_with = None,
        compatible_with = None,
        restricted_to = None,
        testonly = None,
        **kwargs):
    """Generates the code for the static C++ flatbuffer API for the specified fbs file.

    Generates a target of name `name` that can be depended on by C++ code (it
    advertises `CcInfo`) and by other static_flatbuffer rules.

    The generated library consists of one header per source, suffixed with
    `_static.h` and prefixed with the name of the flatbuffer file itself
    (i.e., if you have a src of `foo.fbs`, then the resulting header will be
    `foo_static.h`).

    `:name` advertises both `CcInfo` (re-exported from the underlying
    cc_library) and `StaticFlatbufferReflectionInfo` so codegen aspects
    (e.g. `foxglove_ts_library`) can walk the dep graph. The metadata provider
    exposes one `(bfbs, fbs_name)` entry per source. `deps` may freely mix
    `static_flatbuffer` targets and raw `flatbuffer_cc_library` targets that
    lack the provider; aspects with `required_providers =
    [StaticFlatbufferReflectionInfo]` skip the latter.

    Args:
      name: Target name.
      srcs: List of fbs files to codegen.
      visibility: Desired rule visibility.
      deps: List of flatbuffer dependencies of this rule. Each entry must have
        a sibling `<dep>_fbs` `flatbuffer_cc_library` (which `static_flatbuffer`
        generates automatically).
      target_compatible_with: Optional, the list of constraints the target
        platform must satisfy for this target to be considered compatible.
      compatible_with: Optional, the list of environments this rule can be built
        for, in addition to default-supported environments.
      restricted_to: Optional, the list of environments this rule can be built
        for, instead of default-supported environments.
      testonly: Optional, whether the generated targets are test-only.
    """
    fbs_suffix = "_fbs"

    flatbuffer_cc_library(
        name = name + fbs_suffix,
        srcs = srcs,
        deps = [dep + fbs_suffix for dep in deps],
        gen_reflections = True,
        visibility = visibility,
        target_compatible_with = target_compatible_with,
        compatible_with = compatible_with,
        restricted_to = restricted_to,
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
        target_compatible_with = target_compatible_with,
        compatible_with = compatible_with,
        restricted_to = restricted_to,
        testonly = testonly,
    )

    # `:name` is a custom rule that advertises CcInfo (re-exported from the
    # underlying `:name_cc` cc_library) and StaticFlatbufferReflectionInfo.
    # Codegen aspects walk the dep graph via the rule's
    # `static_flatbuffer_deps` attribute, which only references user-declared
    # deps; this scopes propagation to the static_flatbuffer subgraph and
    # naturally stops at raw `flatbuffer_cc_library` boundaries (which are
    # plain cc_library and don't have that attribute).
    cc_library(
        name = name + "_cc",
        hdrs = header_names,
        deps = [clean_dep("//aos/flatbuffers:static_table"), clean_dep("//aos/flatbuffers:static_vector"), clean_dep("//aos:macros"), name + fbs_suffix] + deps,
        visibility = ["//visibility:private"],
    )
    _static_flatbuffer(
        name = name,
        cc_lib = ":" + name + "_cc",
        bfbs = ":" + reflection_out,
        fbs_names = [src.removeprefix(":") for src in srcs],
        static_flatbuffer_deps = deps,
        visibility = visibility,
        target_compatible_with = target_compatible_with,
        compatible_with = compatible_with,
        restricted_to = restricted_to,
        testonly = testonly,
    )

    native.alias(
        name = name + "_reflection_out",
        actual = name + fbs_suffix + "_reflection_out",
        visibility = visibility,
        compatible_with = compatible_with,
        restricted_to = restricted_to,
        testonly = testonly,
    )
