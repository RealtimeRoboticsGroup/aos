"""Bazel rules for generating Foxglove-compatible TypeScript types from FlatBuffer schemas.

Usage:
    load("@aos//tools/foxglove/ts_generator:foxglove_ts.bzl", "foxglove_ts_library")

    foxglove_ts_library(
        name = "clock_timepoints_foxglove_ts",
        dep = "//aos/util:clock_timepoints_fbs",
    )

The macro automatically walks the static_flatbuffer dependency graph via a Bazel
aspect and generates a TypeScript file for each transitively reachable schema.
There is no need to mirror the FBS dep tree manually.
"""

load("//aos/flatbuffers:generate.bzl", "StaticFlatbufferReflectionInfo")
load("//tools/build_rules:clean_dep.bzl", "clean_dep")

FoxgloveTsInfo = provider(
    doc = "Information about Foxglove TypeScript files generated for the " +
          "schemas owned by a single static_flatbuffer target.",
    fields = {
        "transitive_fbs_to_ts": "depset[struct]: (fbs_name, ts_import_path) " +
                                "pairs for this target and all transitive deps. " +
                                "Consumed by the aspect to emit --import flags " +
                                "covering the full closure, because .fbs files " +
                                "may reference types from transitively-included " +
                                "schemas, not just their direct deps.",
        "transitive_ts_files": "depset[File]: this target's .ts files plus all " +
                               "transitive generated .ts files.",
        "ts_files": "list[File]: .ts files generated for this target's own " +
                    "schemas (one per source .fbs).",
    },
)

def _ts_import_path(ts_file):
    # Emit TS imports relative to the generated .ts file's package. Since all
    # generated files live alongside their source .fbs in bazel-bin, a relative
    # path of './<basename-minus-ext>' works when the consuming panel bundles
    # everything together. Callers relying on a different TS module layout would
    # need to post-process these paths.
    return "./" + ts_file.basename.removesuffix(".ts")

def _package_basename_prefix(label):
    # Build a stable, filesystem-safe prefix from a target's bazel package so
    # that generated .ts basenames don't collide across packages when bundled
    # into a single directory. Without this, two .fbs files with the same
    # basename but different casing (e.g. the foxglove SDK's `Pose.fbs` and our
    # own `pose.fbs`) would clobber each other and trip TypeScript's
    # forceConsistentCasingInFileNames check.
    parts = []
    if label.workspace_name:
        # Under bzlmod the canonical repo name carries the module/extension
        # path (e.g. "+dependencies+com_github_foxglove_schemas" or "aos+"),
        # whose spelling varies between Bazel versions; keep just the last
        # component so the output names stay stable.
        parts.append(label.workspace_name.replace("~", "+").rstrip("+").rsplit("+", 1)[-1])
    if label.package:
        parts.extend(label.package.split("/"))
    return "_".join(parts) + "__" if parts else ""

def _foxglove_ts_aspect_impl(target, ctx):
    info = target[StaticFlatbufferReflectionInfo]

    # Declare one output .ts per owned schema. The basename is prefixed with a
    # sanitized form of the target's package so files from different packages
    # don't collide when symlinked into the same bundle directory.
    prefix = _package_basename_prefix(target.label)
    own = [
        struct(
            schema = schema,
            output = ctx.actions.declare_file(
                prefix +
                schema.fbs_name.removesuffix(".fbs").rsplit("/", 1)[-1] +
                "_foxglove.ts",
            ),
        )
        for schema in info.schemas
    ]

    # Read direct deps from `ctx.rule.attr.static_flatbuffer_deps` rather than
    # the rule-side captured targets: only the aspect's view of those targets
    # carries the aspect-applied FoxgloveTsInfo. Filter to deps that actually
    # have FoxgloveTsInfo, since `static_flatbuffer_deps` may contain raw
    # `flatbuffer_cc_library` targets which `required_providers` skips during
    # aspect application.
    direct_dep_infos = [
        dep[FoxgloveTsInfo]
        for dep in ctx.rule.attr.static_flatbuffer_deps
        if FoxgloveTsInfo in dep
    ]

    # Transitive fbs -> ts_import_path mapping from all deps. We need the full
    # closure (not just direct deps) because .fbs files can reference types
    # declared in transitively-included schemas.
    transitive_fbs_to_ts_list = depset(
        transitive = [dep.transitive_fbs_to_ts for dep in direct_dep_infos],
    ).to_list()

    for entry in own:
        args = ctx.actions.args()
        args.add("--reflection_bfbs", entry.schema.bfbs)
        args.add("--output_file", entry.output)
        args.add("--base_file_name", entry.schema.fbs_name)
        mappings = [
            "%s=%s" % (dep_entry.fbs_name, dep_entry.ts_import_path)
            for dep_entry in transitive_fbs_to_ts_list
            if dep_entry.fbs_name != entry.schema.fbs_name
        ]

        # Add same-node sibling imports so cross-references within a multi-src
        # target (e.g. b.fbs referencing a type in a.fbs) resolve to the
        # sibling .ts file in the same bazel package.
        for sibling in own:
            if sibling.schema.fbs_name != entry.schema.fbs_name:
                mappings.append("%s=%s" % (
                    sibling.schema.fbs_name,
                    _ts_import_path(sibling.output),
                ))
        if mappings:
            args.add("--import", ",".join(mappings))

        ctx.actions.run(
            outputs = [entry.output],
            inputs = [entry.schema.bfbs],
            executable = ctx.executable._generator,
            arguments = [args],
            mnemonic = "FoxgloveTsGen",
            progress_message = "Generating Foxglove TS types for %s" %
                               entry.schema.fbs_name,
        )

    own_outputs = [entry.output for entry in own]
    transitive_ts_files = depset(
        own_outputs,
        transitive = [dep.transitive_ts_files for dep in direct_dep_infos],
    )
    transitive_fbs_to_ts_out = depset(
        [
            struct(
                fbs_name = entry.schema.fbs_name,
                ts_import_path = _ts_import_path(entry.output),
            )
            for entry in own
        ],
        transitive = [dep.transitive_fbs_to_ts for dep in direct_dep_infos],
    )

    return [FoxgloveTsInfo(
        ts_files = own_outputs,
        transitive_ts_files = transitive_ts_files,
        transitive_fbs_to_ts = transitive_fbs_to_ts_out,
    )]

_foxglove_ts_aspect = aspect(
    implementation = _foxglove_ts_aspect_impl,
    # Propagate via the `static_flatbuffer` rule's dedicated propagation
    # attribute rather than `deps` so traversal stays inside the
    # static_flatbuffer subgraph. Plain `cc_library` (multi-source
    # static_flatbuffer outputs, the inner `_cc` library, external
    # `flatbuffer_cc_library`) doesn't have this attribute, so propagation
    # cleanly stops at those boundaries.
    attr_aspects = ["static_flatbuffer_deps"],
    required_providers = [StaticFlatbufferReflectionInfo],
    provides = [FoxgloveTsInfo],
    attrs = {
        "_generator": attr.label(
            default = Label(clean_dep("//tools/foxglove/ts_generator:foxglove_ts_generator")),
            executable = True,
            cfg = "exec",
        ),
    },
)

def _foxglove_ts_library_impl(ctx):
    info = ctx.attr.dep[FoxgloveTsInfo]
    return [
        DefaultInfo(files = info.transitive_ts_files),
        info,
    ]

_foxglove_ts_library_rule = rule(
    implementation = _foxglove_ts_library_impl,
    attrs = {
        "dep": attr.label(
            mandatory = True,
            providers = [StaticFlatbufferReflectionInfo],
            aspects = [_foxglove_ts_aspect],
            doc = "A static_flatbuffer target. The aspect walks its " +
                  "static_flatbuffer dep graph and emits one .ts file per " +
                  "schema in the closure.",
        ),
    },
    provides = [FoxgloveTsInfo],
)

def foxglove_ts_library(name, dep, visibility = None):
    """Generates Foxglove-compatible TypeScript types from a static_flatbuffer target.

    The transitive set of TypeScript files generated for this target's FBS
    dependency closure is exposed via DefaultInfo.

    Args:
        name: Target name.
        dep: A static_flatbuffer target (e.g. "//aos/util:clock_timepoints_fbs").
             The aspect walks its dep graph and emits a .ts file per node.
        visibility: Target visibility.
    """
    _foxglove_ts_library_rule(
        name = name,
        dep = dep,
        visibility = visibility,
    )

def _foxglove_ts_bundle_impl(ctx):
    """Collects all unique .ts files from foxglove_ts_library deps and symlinks them into one directory."""
    seen = {}
    for dep in ctx.attr.deps:
        info = dep[FoxgloveTsInfo]
        for ts_file in info.transitive_ts_files.to_list():
            if ts_file.basename not in seen:
                seen[ts_file.basename] = ts_file

    outputs = []
    for basename, src_file in seen.items():
        out = ctx.actions.declare_file(ctx.attr.out_dir + "/" + basename)
        ctx.actions.symlink(output = out, target_file = src_file)
        outputs.append(out)

    return [DefaultInfo(files = depset(outputs))]

_foxglove_ts_bundle = rule(
    implementation = _foxglove_ts_bundle_impl,
    attrs = {
        "deps": attr.label_list(
            mandatory = True,
            providers = [FoxgloveTsInfo],
            doc = "foxglove_ts_library targets whose transitive .ts files should be bundled.",
        ),
        "out_dir": attr.string(
            mandatory = True,
            doc = "Sub-directory (relative to the package) in which to place the symlinked files.",
        ),
    },
)

def _sanitize_label(label):
    # Turn a label (e.g. "@aos//aos/util:clock_timepoints_fbs" or
    # "//aos/events/event_loop:event_loop_fbs") into a string that is safe to embed in a
    # Bazel target name.
    result = label
    for ch in ("@", "/", ":", "."):
        result = result.replace(ch, "_")
    return result.strip("_")

def foxglove_ts_bundle(name, deps, out_dir, visibility = None):
    """Flattens generated Foxglove TypeScript files from multiple FBS targets into a single directory.

    Since the generated .ts files use basename-only relative imports between each
    other (e.g. ``import { Foo } from './bar_foxglove'``), they must all live in
    the same directory at TypeScript compile time. This macro instantiates a
    private `foxglove_ts_library` for each entry in ``deps``, collects the full
    transitive closure of .ts files from each one, deduplicates by basename, and
    symlinks them into ``out_dir``.

    Args:
        name: Target name.
        deps: List of `static_flatbuffer` targets (typically ``*_fbs``). For
            each entry the macro creates a private `foxglove_ts_library` whose
            aspect walks the FBS dep graph and emits a .ts file per
            transitively-reachable schema.
        out_dir: Output sub-directory relative to the package.
        visibility: Target visibility.
    """
    library_targets = []
    for dep in deps:
        # Embed both the bundle name and a sanitized form of the dep label to
        # keep auto-generated target names unique within the package even when
        # the same FBS basename appears in multiple bundles or packages.
        lib_name = "_%s_%s_foxglove_ts" % (name, _sanitize_label(dep))
        foxglove_ts_library(
            name = lib_name,
            dep = dep,
            visibility = ["//visibility:private"],
        )
        library_targets.append(":" + lib_name)

    _foxglove_ts_bundle(
        name = name,
        deps = library_targets,
        out_dir = out_dir,
        visibility = visibility,
    )
