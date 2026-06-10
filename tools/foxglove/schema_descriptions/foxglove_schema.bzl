"""Bazel macro for generating Foxglove `MessageSchemaDescription` literals from TS types."""

load("@aspect_rules_js//js:defs.bzl", "js_run_binary")

def foxglove_schema_descriptions(name, src, type_names, out, **kwargs):
    """Generates a `.generated.ts` file containing Foxglove schema descriptions.

    The input TypeScript file is parsed and each named type alias is walked structurally to
    produce a `MessageSchemaDescription` constant named `<type_name>Schema`. All exports land
    in a single output file.

    Args:
        name: Bazel target name.
        src: TypeScript source file containing the type aliases to convert.
        type_names: The names of the exported type aliases to convert. At least one is required.
        out: Output path for the generated `.generated.ts` file.
        kwargs: Passed through to the underlying js_run_binary target.
    """

    if not type_names:
        fail("foxglove_schema_descriptions: `type_names` must contain at least one entry.")

    js_run_binary(
        name = name,
        srcs = [src],
        # `Label()` is evaluated when the .bzl is loaded, anchoring this label to the repo that
        # owns the .bzl (i.e. AOS). That keeps the macro portable: it works whether expanded from
        # inside the AOS workspace or from a downstream workspace that pulls AOS in via
        # `local_repository`.
        tool = Label("//tools/foxglove/schema_descriptions:generate_foxglove_schema_description"),
        outs = [out],
        args = [
            "$(execpath {})".format(src),
            "$(execpath {})".format(out),
        ] + type_names,
        **kwargs
    )
