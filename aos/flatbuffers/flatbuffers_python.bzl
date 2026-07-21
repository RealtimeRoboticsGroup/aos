load("@rules_python//python:defs.bzl", "py_library")

FLATC_PATH = "@com_github_google_flatbuffers//:flatc"

FLATC_ARGS = [
    "--python",
    "--require-explicit-ids",
    "--gen-all",
    # Without --python-typing, flatc doesn't add import statements when --gen-all is
    # provided (it's not clear why). Type hints can't hurt, so it's all good.
    "--python-typing",
    "--gen-object-api",
    "--python-fields-snake-case",
]

"""Contains information about a flatbuffers schema and its dependencies.

Fields:
    main: File, the top-level flatbuffers schema file.
    srcs: depset, the set of all dependent schema files. This includes the main file.
"""
FlatbufferPyInfo = provider(
    fields = {
        "main": "Top level source file",
        "srcs": "Transitive source files.",
    },
)

def _flatbuffer_py_srcs_impl(ctx):
    main = ctx.files.src[0]
    srcs = depset([main], transitive = [dep[FlatbufferPyInfo].srcs for dep in ctx.attr.deps])
    return [FlatbufferPyInfo(main = main, srcs = srcs)]

_flatbuffer_py_srcs = rule(
    implementation = _flatbuffer_py_srcs_impl,
    attrs = {
        "deps": attr.label_list(
            default = [],
            providers = [FlatbufferPyInfo],
        ),
        "src": attr.label(
            mandatory = True,
            allow_single_file = [".fbs"],
        ),
    },
)

def _flatbuffer_py_gen_impl(ctx):
    target_info = ctx.attr.target[FlatbufferPyInfo]

    name = ctx.label.name
    if not name.endswith("_gen"):
        fail("Implementation expects target name to end with '_gen'.")
    out_dir_name = name[:name.find("_gen")]
    out_dir = ctx.actions.declare_directory(out_dir_name)

    args = [ctx.executable._flatc.path]
    args.extend(FLATC_ARGS)
    args.extend(["--python-import-prefix", out_dir_name])

    workspaces = []

    # Extract the list of all workspaces.
    for src in target_info.srcs.to_list():
        root = src.owner.workspace_root

        # This makes sure we include the root of the repo in the include path
        # as well as bazel-bin.
        if root == "":
            root = "./"

        if root and root not in workspaces:
            workspaces.append(root)

    for path in workspaces:
        # Include both the workspace and the generated path for each.
        for subpath in ["", ctx.bin_dir.path + "/"]:
            args.append("-I")
            args.append(subpath + path)
    args.append("-I")
    args.append("%s.runfiles/%s" % (
        ctx.executable._flatc.path,
        ctx.executable._flatc.owner.repo_name or "_main",
    ))

    args.extend(["-o", out_dir.path])
    args.append(target_info.main.path)

    ctx.actions.run_shell(
        outputs = [out_dir],
        inputs = target_info.srcs,
        tools = [ctx.executable._flatc],
        command = " ".join(args),
    )

    return [DefaultInfo(runfiles = ctx.runfiles(files = [out_dir]))]

_flatbuffer_py_gen = rule(
    implementation = _flatbuffer_py_gen_impl,
    attrs = {
        "target": attr.label(
            mandatory = True,
            providers = [FlatbufferPyInfo],
        ),
        "_flatc": attr.label(
            executable = True,
            cfg = "exec",
            default = Label(FLATC_PATH),
        ),
    },
)

def flatbuffer_py_library(
        name,
        src,
        deps = [],
        visibility = None,
        target_compatible_with = None):
    """Creates a py_library containing the generated Python modules for a schema.

    When flatc compiles for Python, it reflects the namespace of each object in the form
    of its module hierarchy; i.e. a 'table Baz' in the namespace 'foo.bar' becomes the
    Python module 'foo.bar.Baz' and contains all generated Python objects for
    'table Baz'.

    Moreover, flatc creates a separate Python module for each object (enum, struct,
    table) in the schema, in the interest of preserving the notion of namespaces.

    This means that without examining the contents of the schema, we don't know what the
    outputs of running flatc on the schema is going to be. That doesn't fit in well in
    the Bazel model. To get around this, we declare a directory for the schema in the
    Bazel package that its flatbuffer_py_library target belongs to, and ask flatc to
    write all generated files and folders in there. We can then pass that folder to a
    py_library.

    To keep things simple and avoid having to make too many changes to flatc's code
    generation, we have flatc generate modules for a schema's includes as well. So a
    flatbuffer_py_library is self-contained. This makes imports in the generated code
    straightforward, at the cost of potentially having duplicates of the same module in
    the runfiles (within different parent modules, so they won't clash).
    """
    if not name.endswith("_fbs_py"):
        fail("Target name must be of the form <name>_fbs_py")

    _flatbuffer_py_srcs(
        name = "%s_srcs" % name,
        src = src,
        deps = deps,
        visibility = visibility,
    )

    _flatbuffer_py_gen(
        name = "%s_gen" % name,
        target = ":%s_srcs" % name,
        visibility = visibility,
    )

    py_library(
        name = name,
        data = [
            ":%s_gen" % name,
        ],
        imports = [
            ".",
        ],
        deps = [
            # This is wrapped in `Label()` so that the string is parsed relative
            # to this file's repo (i.e. @aos), and not relative to the repo of
            # the caller. This lets us use the `flatbuffer_py_library` macro in
            # external repos.
            Label("//aos/flatbuffers:flatpy"),
        ],
        visibility = visibility,
        target_compatible_with = target_compatible_with,
    )

flatbuffer_py_srcs = _flatbuffer_py_srcs
