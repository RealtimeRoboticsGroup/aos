load("//tools/build_rules:label.bzl", "expand_label")
load("//tools/build_rules:select.bzl", "address_size_select")

AosConfigInfo = provider(fields = [
    "transitive_flatbuffers",
    "transitive_src",
])

def aos_config(name, src, flatbuffers = [], deps = [], visibility = None, testonly = False, target_compatible_with = None):
    if src.split(":")[-1].split("/")[-1] == name + ".json":
        fail(("aos_config(name = %r): the target name must differ from the src " +
              "basename, because the flattened config is written to %r. " +
              "Rename the target (e.g. name = %r).") % (name, name + ".json", name + "_config"))

    _aos_config(
        name = name,
        src = src,
        flags = address_size_select({
            "32": ["--max_queue_size_override=0xffff"],
            "64": ["--max_queue_size_override=0xffffffff"],
        }),
        config_json = name + ".json",
        config_stripped = name + ".stripped.json",
        config_binary = name + ".bfbs",
        deps = deps,
        flatbuffers = [expand_label(flatbuffer) + "_reflection_out" for flatbuffer in flatbuffers],
        visibility = visibility,
        testonly = testonly,
        target_compatible_with = target_compatible_with,
    )

def _aos_config_impl(ctx):
    config = ctx.outputs.config_json
    stripped_config = ctx.outputs.config_stripped
    binary_config = ctx.outputs.config_binary

    flatbuffers_depset = depset(
        ctx.files.flatbuffers,
        transitive = [dep[AosConfigInfo].transitive_flatbuffers for dep in ctx.attr.deps],
    )

    src_depset = depset(
        ctx.files.src,
        transitive = [dep[AosConfigInfo].transitive_src for dep in ctx.attr.deps],
    )

    all_files = flatbuffers_depset.to_list() + src_depset.to_list()
    ctx.actions.run(
        outputs = [stripped_config, binary_config],
        inputs = all_files,
        arguments = ctx.attr.flags + [
            "--stripped_output=" + stripped_config.path,
            "--binary_output=" + binary_config.path,
            (ctx.label.workspace_root or ".") + "/" + ctx.files.src[0].short_path,
            ctx.bin_dir.path,
        ] + [f.path for f in flatbuffers_depset.to_list()],
        progress_message = "Flattening config",
        executable = ctx.executable._config_flattener,
    )
    ctx.actions.run(
        outputs = [config],
        inputs = all_files,
        arguments = ctx.attr.flags + [
            "--full_output=" + config.path,
            (ctx.label.workspace_root or ".") + "/" + ctx.files.src[0].short_path,
            ctx.bin_dir.path,
        ] + [f.path for f in flatbuffers_depset.to_list()],
        progress_message = "Flattening config",
        executable = ctx.executable._config_flattener,
    )
    runfiles = ctx.runfiles(files = [stripped_config, binary_config])
    return [
        DefaultInfo(
            files = depset([stripped_config, binary_config]),
            runfiles = runfiles,
        ),
        AosConfigInfo(
            transitive_flatbuffers = flatbuffers_depset,
            transitive_src = src_depset,
        ),
    ]

_aos_config = rule(
    attrs = {
        "config_binary": attr.output(mandatory = True),
        "config_json": attr.output(mandatory = True),
        "config_stripped": attr.output(mandatory = True),
        "deps": attr.label_list(
            providers = [AosConfigInfo],
        ),
        "flags": attr.string_list(
            doc = "Additional flags to pass to config_flattener.",
        ),
        "flatbuffers": attr.label_list(
            mandatory = False,
        ),
        "src": attr.label(
            mandatory = True,
            allow_files = True,
        ),
        "_config_flattener": attr.label(
            executable = True,
            cfg = "exec",
            default = Label("//aos:config_flattener"),
        ),
    },
    doc = "Checks and flattens an AOS config, including embedding flatbuffer schemas.",
    implementation = _aos_config_impl,
)

def aos_config_flatbuffers(name, flatbuffers = [], deps = [], visibility = None, testonly = False, target_compatible_with = None):
    _aos_config_flatbuffers(
        name = name,
        deps = deps,
        flatbuffers = [expand_label(flatbuffer) + "_reflection_out" for flatbuffer in flatbuffers],
        visibility = visibility,
        testonly = testonly,
        target_compatible_with = target_compatible_with,
    )

def _aos_config_flatbuffers_impl(ctx):
    flatbuffers_depset = depset(
        ctx.files.flatbuffers,
        transitive = [dep[AosConfigInfo].transitive_flatbuffers for dep in ctx.attr.deps],
    )
    return [AosConfigInfo(transitive_flatbuffers = flatbuffers_depset, transitive_src = depset())]

_aos_config_flatbuffers = rule(
    attrs = {
        "deps": attr.label_list(
            providers = [AosConfigInfo],
        ),
        "flatbuffers": attr.label_list(
            mandatory = False,
        ),
    },
    doc = "A group of flatbuffer schemas to embed in an AOS config (without an attached config yet).",
    implementation = _aos_config_flatbuffers_impl,
)
