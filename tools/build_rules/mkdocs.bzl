load("@rules_python//python:defs.bzl", "py_binary")
load("//tools/build_rules:clean_dep.bzl", "clean_dep")

def _mkdocs_impl(ctx):
    output_file = ctx.actions.declare_file(ctx.attr.name + ".tar")

    # mkdocs only looks for markdown in the docs/ directory next to the
    # config, so generated srcs (which live in bazel-out) would be silently
    # left out.  Stage the config and all the srcs into one tree instead.
    root = ctx.attr.name + "_root/"
    config = ctx.actions.declare_file(root + "mkdocs.yaml")
    ctx.actions.symlink(output = config, target_file = ctx.file.config)
    staged_srcs = []
    seen = {}
    for src in ctx.files.srcs:
        if src.basename in seen:
            fail("Duplicate markdown file name %s: %s and %s" % (src.basename, seen[src.basename].short_path, src.short_path))
        seen[src.basename] = src
        staged = ctx.actions.declare_file(root + "docs/" + src.basename)
        ctx.actions.symlink(output = staged, target_file = src)
        staged_srcs.append(staged)

    build_args = ctx.actions.args()
    build_args.add("build")
    site_subdir = "site"
    build_args.add("-f")
    build_args.add(config)
    build_args.add("--site-dir")
    build_args.add(site_subdir)
    site_dir = config.dirname + "/" + site_subdir
    ctx.actions.run(
        mnemonic = "MkDocsBuild",
        executable = ctx.executable._mkdocs,
        arguments = [build_args],
        inputs = staged_srcs + [config],
        env = {"OUTPUT": output_file.path, "SITE_DIR": site_dir},
        outputs = [output_file],
    )
    return [DefaultInfo(files = depset([output_file]))]

_mkdocs = rule(
    implementation = _mkdocs_impl,
    attrs = {
        "config": attr.label(
            doc = "mkdocs.yaml configuration file to use for the documentation.",
            allow_single_file = [".yaml"],
            mandatory = True,
        ),
        "srcs": attr.label_list(
            allow_files = [".md"],
            doc = "A list of markdown files to generate docs for.",
        ),
        "_mkdocs": attr.label(default = Label("//documentation:mkdocs_bin"), executable = True, cfg = "exec"),
    },
)

def mkdocs(name, srcs, config, **kwargs):
    """Bazel rule to build and serve mkdocs documentation.

    Build a tarball of the HTML files to be served by building "name", or get
    the functionality of `mkdocs serve` by doing a bazel run on "name.serve".

    Args:
      name: name of the rule.
      srcs: A list of markdown files to generate documentation from.
      config: A .yaml specifying the mkdocs configuration to use.
        See https://www.mkdocs.org/user-guide/configuration/
        Note that mkdocs does not allow the mkdocs.yaml configuration to be
        in the same folder as the markdown files (canonically, if the
        mkdocs.yaml is at foo/mkdocs.yaml, the markdown files will be
        at foo/docs/*.md).
    """
    _mkdocs(name = name, srcs = srcs, config = config, **kwargs)
    py_binary(
        name = name + ".serve",
        srcs = [clean_dep("//documentation:mkdocs_bin.py")],
        main = "mkdocs_bin.py",
        args = ["serve", "-f", "$(location %s)" % (config,)],
        deps = ["@pip_deps//mkdocs"],
        data = srcs + [config],
        **kwargs
    )
