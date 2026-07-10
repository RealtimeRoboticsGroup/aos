# Description:
#   BUILD rules for generating flatbuffer files in various languages.

"""Rules for building flatbuffers with Bazel."""

load("@aos//tools/build_rules:clean_dep.bzl", "clean_dep")
load("@io_bazel_rules_go//go:def.bzl", "go_library")
load("@rules_cc//cc:defs.bzl", "cc_library")
load("@rules_rust//rust:defs.bzl", "rust_library")
load("@rules_rust//rust:rust_common.bzl", "CrateInfo")

flatc_path = "@com_github_google_flatbuffers//:flatc"

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

def flatbuffer_cc_library(
        name,
        srcs,
        srcs_filegroup_name = "",
        output_folder = "",
        deps = [],
        includes = [],
        include_paths = DEFAULT_INCLUDE_PATHS,
        cc_include_paths = [],
        flatc_args = DEFAULT_FLATC_ARGS,
        visibility = None,
        compatible_with = None,
        restricted_to = None,
        target_compatible_with = None,
        srcs_filegroup_visibility = None,
        gen_reflections = False):
    """A cc_library with the generated reader/writers for the given flatbuffer definitions.

    Args:
      name: Rule name.
      srcs: Source .fbs files. Sent in order to the compiler.
      srcs_filegroup_name: Name of the output filegroup that holds srcs. Pass this
          filegroup into the `includes` parameter of any other
          flatbuffer_cc_library that depends on this one's schemas.
      deps: Optional, list of other flatbuffer_cc_library's to depend on. Cannot be specified
          alongside includes.
      includes: Optional, list of filegroups of schemas that the srcs depend on.
          Use of this is discouraged, and may be deprecated.
      include_paths: Optional, list of paths the includes files can be found in.
      cc_include_paths: Optional, list of paths to add to the cc_library includes attribute.
      flatc_args: Optional list of additional arguments to pass to flatc
          (e.g. --gen-mutable).
      visibility: The visibility of the generated cc_library. By default, use the
          default visibility of the project.
      target_compatible_with: Optional, the list of constraints the target
        platform must satisfy for this target to be considered compatible.
      srcs_filegroup_visibility: The visibility of the generated srcs filegroup.
          By default, use the value of the visibility parameter above.
      gen_reflections: Optional, if true this will generate the flatbuffer
        reflection binaries for the schemas.
      compatible_with: Optional, The list of environments this rule can be built
        for, in addition to default-supported environments.
      restricted_to: Optional, The list of environments this rule can be built
        for, instead of default-supported environments.
      target_compatible_with: Optional, The list of target platform constraints
        to use.

    This produces:
      filegroup([name]_srcs): all generated .h files.
      filegroup(srcs_filegroup_name if specified, or [name]_includes if not):
          Other flatbuffer_cc_library's can pass this in for their `includes`
          parameter, if they depend on the schemas in this library.
      Fileset([name]_reflection): (Optional) all generated reflection binaries.
      cc_library([name]): library with sources and flatbuffers deps.
    """
    if deps and includes:
        # There is no inherent reason we couldn't support both, but this discourages
        # use of includes without good reason.
        fail("Cannot specify both deps and include in flatbuffer_cc_library.")
    if deps:
        includes = [d + "_srcs" for d in deps]
    reflection_name = "%s_reflection" % name if gen_reflections else ""

    srcs_lib = srcs_filegroup_name or "%s_srcs" % (name)
    flatbuffer_library_public(
        name = srcs_lib,
        srcs = srcs,
        output_suffix = "_generated.h",
        language_flag = "-c",
        output_folder = output_folder,
        deps = includes,
        include_paths = include_paths,
        flatc_args = flatc_args,
        compatible_with = compatible_with,
        restricted_to = restricted_to,
        target_compatible_with = target_compatible_with,
        reflection_name = reflection_name,
        reflection_visibility = visibility,
        visibility = srcs_filegroup_visibility if srcs_filegroup_visibility != None else visibility,
    )
    cc_library(
        name = name,
        hdrs = [
            ":" + srcs_lib,
        ],
        srcs = [
            ":" + srcs_lib,
        ],
        features = [
            "-parse_headers",
        ],
        deps = [
            clean_dep("@com_github_google_flatbuffers//:runtime_cc"),
            clean_dep("@com_github_google_flatbuffers//:flatbuffers"),
        ] + deps,
        includes = cc_include_paths,
        compatible_with = compatible_with,
        restricted_to = restricted_to,
        target_compatible_with = target_compatible_with,
        linkstatic = 1,
        visibility = visibility,
    )

def flatbuffer_go_library(
        name,
        srcs,
        importpath,
        compatible_with = None,
        target_compatible_with = None,
        includes = [],
        include_paths = DEFAULT_INCLUDE_PATHS,
        flatc_args = DEFAULT_FLATC_GO_ARGS,
        visibility = None,
        srcs_filegroup_visibility = None):
    srcs_lib = "%s_srcs" % (name)
    flatc_args = flatc_args + ["--go-namespace", importpath.split("/")[-1]]

    flatbuffer_library_public(
        name = srcs_lib,
        srcs = srcs,
        output_suffix = "_generated.go",
        language_flag = "--go",
        deps = includes,
        include_paths = include_paths,
        flatc_args = flatc_args,
        compatible_with = compatible_with,
        target_compatible_with = target_compatible_with,
        visibility = srcs_filegroup_visibility if srcs_filegroup_visibility != None else ["//visibility:private"],
    )
    go_library(
        name = name,
        srcs = [srcs_lib],
        deps = [clean_dep("@com_github_google_flatbuffers//go")],
        importpath = importpath,
        visibility = visibility,
        compatible_with = compatible_with,
        target_compatible_with = target_compatible_with,
    )

def _flatbuffer_rust_lib_gen_impl(ctx):
    # TODO(Brian): I think this needs changes to properly handle multiple .fbs files in a rule.
    uses = []
    for (dep, dep_srcs) in zip(ctx.attr.deps, ctx.attr.dep_srcs):
        for dep_src in dep_srcs[FlatbufferLibraryInfo].srcs.to_list():
            uses.append((dep[CrateInfo].name, dep_src.basename.replace(".fbs", "_generated")))
    lib_rs_content = "\n".join(
        [
            "// Automatically generated by the Flatbuffers Bazel rules. Do not modify",
            "#![allow(unused_imports)]",
        ] + ["use %s as %s;" % (crate, use_as) for (crate, use_as) in uses] +
        ["include!(\"%s\");" % src.basename for src in ctx.files.srcs_lib],
    )
    output = ctx.actions.declare_file(ctx.attr.name + "_lib.rs")
    ctx.actions.write(
        output = output,
        content = lib_rs_content,
    )
    return [DefaultInfo(files = depset([output]))]

"""Generates a lib.rs for a flatbuffer_rust_library.

flatc generates individual .rs files for us. It can also generate a top-level mod.rs to be included
in a crate, but that is laid out to include all flatbuffers files in a project. That's not a good
fit for Bazel rules and monorepos, so we generate an alternative that imports all dependencies under
their expected names."""
_flatbuffer_rust_lib_gen = rule(
    implementation = _flatbuffer_rust_lib_gen_impl,
    attrs = {
        "dep_srcs": attr.label_list(mandatory = True, providers = [FlatbufferLibraryInfo], doc = "The _srcs rules for all our direct dependencies"),
        "deps": attr.label_list(mandatory = True, providers = [CrateInfo]),
        "srcs_lib": attr.label(mandatory = True, doc = "The generated srcs for this rule"),
    },
)

def flatbuffer_rust_library(
        name,
        srcs,
        compatible_with = None,
        target_compatible_with = select({
            "//conditions:default": [clean_dep("//tools/platforms/rust:has_support")],
            clean_dep("//tools:has_msan"): ["@platforms//:incompatible"],
        }),
        deps = [],
        include_paths = DEFAULT_INCLUDE_PATHS,
        flatc_args = DEFAULT_FLATC_RUST_ARGS,
        include_reflection = True,
        crate_name = None,
        visibility = None,
        srcs_filegroup_visibility = None):
    includes = [d + "_includes" for d in deps]
    srcs_lib = "%s_srcs" % (name)
    lib_gen = "%s_lib_gen" % (name)
    deps = list(deps)
    if include_reflection:
        deps.append(clean_dep("//aos/flatbuffers/reflection:reflection_rust_fbs"))

    flatbuffer_library_public(
        name = srcs_lib,
        srcs = srcs,
        language_flag = "--rust",
        output_suffix = "_generated.rs",
        deps = includes,
        include_paths = include_paths,
        flatc_args = flatc_args,
        compatible_with = compatible_with,
        target_compatible_with = target_compatible_with,
        visibility = srcs_filegroup_visibility if srcs_filegroup_visibility != None else visibility,
    )
    _flatbuffer_rust_lib_gen(
        name = lib_gen,
        deps = deps,
        dep_srcs = [dep + "_srcs" for dep in deps],
        srcs_lib = srcs_lib,
        visibility = ["//visibility:private"],
        compatible_with = compatible_with,
        target_compatible_with = target_compatible_with,
    )
    rust_library(
        name = name,
        srcs = [srcs_lib, lib_gen],
        crate_root = lib_gen,
        crate_name = crate_name,
        deps = ["@com_github_google_flatbuffers//rust:flatbuffers"] + deps,
        edition = "2018",
        visibility = visibility,
        compatible_with = compatible_with,
        target_compatible_with = target_compatible_with,
    )
