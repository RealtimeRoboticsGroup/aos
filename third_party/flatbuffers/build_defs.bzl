# Description:
#   BUILD rules for generating flatbuffer files in various languages.

"""
Rules for building C++ flatbuffers with Bazel.

AOS Note: These have diverged substantially from upstream; they should
probably just be extracted from the third_party/flatbuffers folder entirely.
"""

load("@rules_cc//cc:defs.bzl", "cc_library")
load("//:repo_name.bzl", "repo_name")

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

    all_files = depset(ctx.attr.generated_files)
    if ctx.attr.generated_files:
        outs = ctx.outputs.generated_files

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

        arguments = [prefix + ctx.executable._flatc.path]
        for path in ctx.attr.include_paths + workspaces:
            for subpath in ["", ctx.bin_dir.path + "/"]:
                arguments.append("-I")
                arguments.append(prefix + subpath + path)
        arguments.append("-I")
        arguments.append(prefix + "%s.runfiles/%s" % (ctx.executable._flatc.path, repo_name()))
        arguments.extend(ctx.attr.flatc_args)
        arguments.extend(ctx.attr.language_flags)
        if prefix:
            arguments.extend(["--bfbs-filenames", prefix + "/"])

        arguments.extend([
            "-o",
            prefix + out_dir,
        ])
        arguments.append(src_path)
        if root_folder != None:
            commands.append("(cd " + root_folder + " && " + " ".join(arguments) + ")")
        else:
            commands.append("  ".join(arguments))

    ctx.actions.run_shell(
        outputs = outs,
        inputs = all_srcs,
        tools = [ctx.executable._flatc],
        command = " && ".join(commands),
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
    """Generates code files for reading/writing the given flatbuffers in the
    requested language using the public compiler.

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

    srcs_lib = "%s_srcs" % (name)
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
        visibility = visibility,
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
            "@com_github_google_flatbuffers//:runtime_cc",
            "@com_github_google_flatbuffers//:flatbuffers",
        ] + deps,
        includes = cc_include_paths,
        compatible_with = compatible_with,
        restricted_to = restricted_to,
        target_compatible_with = target_compatible_with,
        linkstatic = 1,
        visibility = visibility,
    )

