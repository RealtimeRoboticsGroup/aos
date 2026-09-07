# Description:
#   The language-agnostic core of the flatbuffer rules: the codegen rule
#   (flatbuffer_library_public), its provider, and the default flatc arguments.
#   The per-language wrappers live next door in cc.bzl, rust.bzl, go.bzl, and
#   python.bzl, so a consumer only loads the rulesets for the
#   languages it actually generates.

"""Core rules for building flatbuffers with Bazel."""

flatc_path = "@aos_flatbuffers//:flatc"

DEFAULT_INCLUDE_PATHS = [
    "./",
]

DEFAULT_FLATC_ARGS = [
    "--gen-object-api",
    "--gen-compare",
    "--keep-prefix",
    "--bfbs-builtins",
    "--bfbs-comments",
    "--cpp-std",
    "c++17",
    "--require-explicit-ids",
    "--gen-mutable",
    "--reflect-names",
    "--cpp-ptr-type",
    "flatbuffers::unique_ptr",
    "--force-empty",
    "--scoped-enums",
    "--gen-name-strings",
]

DEFAULT_FLATC_GO_ARGS = [
    "--gen-onefile",
    "--gen-object-api",
    "--require-explicit-ids",
]

DEFAULT_FLATC_RUST_ARGS = [
    "--gen-object-api",
    "--require-explicit-ids",
    "--gen-name-strings",
]

"""Contains information about a set of flatbuffers which have their code for
reading/writing generated in a single library-style rule.

Fields:
    srcs: [File], the .fbs source files
"""
FlatbufferLibraryInfo = provider()

def _get_flatbuffer_src_root_folder_and_path(src):
    """Helper to get a consistent root folder and path for generated and non-generated flatbuffers inside and outside the build repo"""

    # For flatbuffers built in external repos, we don't want "external/foo" in the path.
    # That will trigger #include "external/foo/bar_generated.h".  To fix that, cd into
    # external/foo, and then add ../../ in front of all paths.
    #
    # We also need to be very careful about what path we feed flatc.  That is the path that
    # it will encode in the reflection flatbuffer.  If someone generates a flatbuffer, we
    # don't want bazel-out/... to be included.
    #
    # This means we have 4 options:
    #
    #  # Non-generated from build repo
    #  src.path ->       path/to/message.fbs
    #  src.short_path -> path/to/message.fbs
    #  src.root ->       ''
    #
    # OR
    #
    #  # Generated from build repo
    #  src.path ->       bazel-out/k8-fastbuild/bin/path/to/message.fbs
    #  src.short_path -> path/to/message.fbs
    #  src.root ->       bazel-out/k8-fastbuild/bin
    #
    # OR
    #
    #  # Non-generated from another repo
    #  src.path ->       external/otherrepo/path/to/message.fbs
    #  src.short_path -> ../otherrepo/path/to/message.fbs
    #  src.root ->       ''
    #
    # OR
    #
    #  # Generated from another repo
    #  src.path ->       bazel-out/k8-opt/bin/external/otherrepo/path/to/message.fbs
    #  src.short_path -> ../otherrepo/path/to/message.fbs
    #  src.root ->       bazel-out/k8-opt/bin
    #
    # root_folder -> the thing before the path.
    # src_path -> the path relative to the repo root.
    #
    input_dir = "/".join(src.short_path.split("/")[:-1])
    root_folder = None
    is_from_another_repo = input_dir.startswith("../")
    if is_from_another_repo:
        second_slash_index = input_dir.find("/", len("../"))

        # Handle flatbuffers in the root of the repo.  Don't want to strip off the last character...
        if second_slash_index == -1:
            root_folder = "external/" + input_dir[3:]
        else:
            root_folder = "external/" + input_dir[3:second_slash_index]

    is_generated = src.root.path != ""
    if is_generated:
        if not root_folder:
            root_folder = src.root.path
        else:
            root_folder = src.root.path + "/" + root_folder

    if root_folder != None:
        prefix = "".join(["../" for _ in root_folder.split("/")])
        src_path = src.path[len(root_folder) + 1:]
    else:
        prefix = ""
        src_path = src.path

    return root_folder, src_path, prefix

def _flatbuffer_library_compile_impl(ctx):
    outs = []
    commands = []
    all_srcs = depset(ctx.files.srcs, transitive = [dep[FlatbufferLibraryInfo].srcs for dep in ctx.attr.deps])

    workspaces = []

    for dep in ctx.attr.deps:
        for dep_src in dep[FlatbufferLibraryInfo].srcs.to_list():
            root = dep_src.owner.workspace_root
            if root and root not in workspaces:
                workspaces.append(root)

    if ctx.attr.generated_files:
        outs = ctx.outputs.generated_files

    has_root_folder = False

    for src in ctx.files.srcs:
        if ctx.attr.generated_files:
            root_folder = None
            src_path = src.path
            prefix = ""
            out_dir = ctx.bin_dir.path + "/" + ctx.label.workspace_root + "/" + ctx.label.package + "/" + ctx.attr.output_folder
        else:
            root_folder, src_path, prefix = _get_flatbuffer_src_root_folder_and_path(src)
            out = ctx.actions.declare_file(ctx.attr.output_folder + src.basename.replace(".fbs", "") + ctx.attr.output_suffix)
            out_dir = out.dirname
            outs.append(out)

        execroot_prefix = prefix
        if root_folder != None:
            # On Windows, Bazel uses directory junctions for external repositories.
            # Changing directory into a junction (e.g. via `cd root_folder`) causes
            # Windows' path resolution to follow the physical target path of the junction
            # when handling parent traversal (`..`), which breaks relative prefixes like `../../`.
            # To resolve this cleanly on both Windows and Linux, we use absolute paths
            # prefixed with `$EXECROOT/` where the shell environment variable `$EXECROOT`
            # is set to the workspace's root directory at runtime.
            has_root_folder = True
            execroot_prefix = "$EXECROOT/"

        arguments = [execroot_prefix + ctx.executable._flatc.path]
        for path in ctx.attr.include_paths + workspaces:
            for subpath in ["", ctx.bin_dir.path + "/"]:
                arguments.append("-I")
                arguments.append(execroot_prefix + subpath + path)
        arguments.append("-I")
        arguments.append(execroot_prefix + "%s.runfiles/%s" % (
            ctx.executable._flatc.path,
            ctx.executable._flatc.owner.repo_name or "_main",
        ))
        arguments.extend(ctx.attr.flatc_args)
        arguments.extend(ctx.attr.language_flags)
        if prefix:
            arguments.extend(["--bfbs-filenames", prefix + "/"])

        arguments.extend([
            "-o",
            execroot_prefix + out_dir,
        ])
        arguments.append(src_path)
        if root_folder != None:
            commands.append("(cd " + root_folder + " && " + " ".join(arguments) + ")")
        else:
            commands.append("  ".join(arguments))

    command_str = " && ".join(commands)
    if has_root_folder:
        command_str = "EXECROOT=$(pwd) && " + command_str

    ctx.actions.run_shell(
        outputs = outs,
        inputs = all_srcs,
        tools = [ctx.executable._flatc],
        command = command_str,
        mnemonic = "Flatc",
        progress_message = "Generating flatbuffer files for %{input}:",
    )
    return [DefaultInfo(files = depset(outs)), FlatbufferLibraryInfo(srcs = all_srcs)]

_flatbuffer_library_compile = rule(
    implementation = _flatbuffer_library_compile_impl,
    attrs = {
        "deps": attr.label_list(default = [], providers = [FlatbufferLibraryInfo], doc = "All of our direct dependencies."),
        "flatc_args": attr.string_list(default = []),
        "generated_files": attr.output_list(mandatory = False),
        "include_paths": attr.string_list(default = []),
        "language_flags": attr.string_list(mandatory = True),
        "output_folder": attr.string(default = ""),
        "output_suffix": attr.string(default = ""),
        "srcs": attr.label_list(mandatory = True, allow_files = True),
        "_flatc": attr.label(executable = True, cfg = "exec", default = Label(flatc_path)),
    },
)

def flatbuffer_library_public(
        name,
        srcs,
        output_suffix,
        language_flag,
        generated_files = None,
        output_folder = "",
        deps = [],
        include_paths = DEFAULT_INCLUDE_PATHS,
        flatc_args = DEFAULT_FLATC_ARGS,
        reflection_name = "",
        reflection_visibility = None,
        compatible_with = None,
        restricted_to = None,
        target_compatible_with = None,
        output_to_bindir = False,
        visibility = None):
    """Generates code files for reading/writing flatbuffers in the requested language.

    Args:
      name: Rule name.
      srcs: Source .fbs files. Sent in order to the compiler.
      output_suffix: Suffix for output files from flatc.
      language_flag: Target language flag. One of [-c, -j, -js].
      deps: Optional, list of filegroups of schemas that the srcs depend on.
      include_paths: Optional, list of paths the includes files can be found in.
      flatc_args: Optional, list of additional arguments to pass to flatc.
      reflection_name: Optional, if set this will generate the flatbuffer
        reflection binaries for the schemas.
      reflection_visibility: The visibility of the generated reflection Fileset.
      output_to_bindir: Passed to genrule for output to bin directory.
      compatible_with: Optional, The list of environments this rule can be
        built for, in addition to default-supported environments.
      restricted_to: Optional, The list of environments this rule can be built
        for, instead of default-supported environments.
      target_compatible_with: Optional, The list of target platform constraints
        to use.
      output_to_bindir: Passed to genrule for output to bin directory.


    This rule creates a filegroup(name) with all generated source files, and
    optionally a Fileset([reflection_name]) with all generated reflection
    binaries.
    """
    if output_to_bindir:
        fail("output_to_bindir is not supported by flatbuffer_library_public.")

    _flatbuffer_library_compile(
        name = name,
        srcs = srcs,
        output_suffix = output_suffix,
        output_folder = output_folder,
        language_flags = [language_flag],
        deps = deps,
        include_paths = include_paths,
        flatc_args = flatc_args,
        generated_files = generated_files,
        compatible_with = compatible_with,
        target_compatible_with = target_compatible_with,
        restricted_to = restricted_to,
        visibility = visibility,
    )

    if reflection_name:
        _flatbuffer_library_compile(
            name = "%s_out" % reflection_name,
            srcs = srcs,
            output_suffix = ".bfbs",
            language_flags = ["-b", "--schema"],
            deps = deps,
            include_paths = include_paths,
            flatc_args = flatc_args,
            compatible_with = compatible_with,
            target_compatible_with = target_compatible_with,
            restricted_to = restricted_to,
            visibility = reflection_visibility,
        )
