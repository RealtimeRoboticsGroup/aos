"""Build a synthetic CUDA toolkit root that mirrors a real CUDA install,
optionally redirecting the host tools in `bin/` through a wrapper
script.

Clang's CUDA driver hard-codes the layout of `--cuda-path`: include/
must live next to nvvm/libdevice/ next to bin/ptxas next to
bin/fatbinary. We can't move any of those around, but we can either:

  * Substitute `bin/ptxas`, `bin/fatbinary`, `bin/nvlink`, and
    `bin/bin2c` with copies of a wrapper (`wrapper` attr set). This
    is what the macOS path does so it can intercept every host-tool
    invocation and bounce it through Rosetta-in-vfkit.

  * Or symlink straight to the real binaries (`wrapper = None`). The
    Linux path uses this — the only reason we still build a synth
    root is that the aarch64 CUDA install ships fatbinary under
    `aarch64-unknown-linux-gnu-fatbinary` and clang demands plain
    `bin/fatbinary`. One rename symlink is much cheaper than booting
    a VM.
"""

# Tools we want to intercept. Keys are the basenames clang invokes;
# values are the matching basename inside the source sysroot's bin/
# directory (the aarch64 CUDA install bundles `fatbinary` under a
# target-prefixed name).
_INTERCEPTED_TOOLS = {
    "bin2c": "bin2c",
    "fatbinary": "aarch64-unknown-linux-gnu-fatbinary",
    "nvlink": "nvlink",
    "ptxas": "ptxas",
}

def _synth_cuda_root_impl(ctx):
    cuda_root = ctx.attr.cuda_root_subdir

    outputs = []

    # The four host tools clang invokes from `<cuda-path>/bin/`. Each
    # gets a copy of the wrapper that dispatches by `basename "$0"`.
    declared_outputs = {
        "bin/bin2c": ctx.outputs.bin2c,
        "bin/fatbinary": ctx.outputs.fatbinary,
        "bin/nvlink": ctx.outputs.nvlink,
        "bin/ptxas": ctx.outputs.ptxas,
        "nvvm/libdevice/libdevice.10.bc": ctx.outputs.libdevice,
    }

    wrapper = ctx.file.wrapper

    # When wrapping, every intercepted tool gets a copy of the wrapper
    # script. When passing through, we need to find the actual real
    # binary in src_files and symlink to it (with a rename for
    # fatbinary).
    real_bin_relpaths = {
        tool_name: "{}/bin/{}".format(cuda_root, src_name)
        for tool_name, src_name in _INTERCEPTED_TOOLS.items()
    }
    real_bin_files = {}
    if wrapper == None:
        for f in ctx.files.src_files:
            sp = f.short_path
            for tool_name, rel in real_bin_relpaths.items():
                if sp.endswith("/" + rel) or sp == rel:
                    real_bin_files[tool_name] = f
                    break
        missing = [t for t in _INTERCEPTED_TOOLS if t not in real_bin_files]
        if missing:
            fail("synth_cuda_root: wrapper=None but couldn't find " +
                 "real binaries for: {}".format(missing))

    for tool_name in _INTERCEPTED_TOOLS.keys():
        out = declared_outputs["bin/" + tool_name]
        if wrapper != None:
            ctx.actions.run_shell(
                inputs = [wrapper],
                outputs = [out],
                command = "cp {src} {dst} && chmod +x {dst}".format(
                    src = wrapper.path,
                    dst = out.path,
                ),
                mnemonic = "SynthCudaWrapper",
            )
        else:
            ctx.actions.symlink(output = out, target_file = real_bin_files[tool_name])
        outputs.append(out)

    intercepted_paths = {
        "{}/bin/{}".format(cuda_root, src_name): True
        for src_name in _INTERCEPTED_TOOLS.values()
    }
    libdevice_rel = "nvvm/libdevice/libdevice.10.bc"

    # `f.short_path` is `<repo-relative path>` for files in this repo
    # and `../<repo_name>/<repo-relative path>` for external repos, so
    # we search for the cuda_root prefix anywhere in the path and take
    # whatever follows it as the relative path inside the synth root.
    needle = "/" + cuda_root + "/"
    for f in ctx.files.src_files:
        sp = f.short_path
        if sp.startswith(cuda_root + "/"):
            rel = sp[len(cuda_root) + 1:]
        elif needle in sp:
            rel = sp.split(needle, 1)[1]
        else:
            continue
        if "{}/{}".format(cuda_root, rel) in intercepted_paths:
            continue
        if rel == libdevice_rel:
            # Use the declared output so cuda_toolkit_info(libdevice=...)
            # can address it by label.
            ctx.actions.symlink(output = ctx.outputs.libdevice, target_file = f)
            outputs.append(ctx.outputs.libdevice)
            continue
        out = ctx.actions.declare_file("{}/{}".format(ctx.attr.out_subdir, rel))
        ctx.actions.symlink(output = out, target_file = f)
        outputs.append(out)

    return [DefaultInfo(files = depset(outputs), runfiles = ctx.runfiles(files = outputs))]

synth_cuda_root = rule(
    implementation = _synth_cuda_root_impl,
    doc = "Mirror a CUDA toolkit dir, replacing host-tool binaries with a wrapper.",
    attrs = {
        "bin2c": attr.output(mandatory = True),
        "cuda_root_subdir": attr.string(
            mandatory = True,
            doc = "Path under src_files's repo that holds the CUDA root, e.g. `usr/local/cuda-12.6`.",
        ),
        "fatbinary": attr.output(mandatory = True),
        "libdevice": attr.output(mandatory = True),
        "nvlink": attr.output(mandatory = True),
        "out_subdir": attr.string(
            mandatory = True,
            doc = "Subdirectory (relative to this package) for the synthetic root's symlink-only outputs.",
        ),
        # Declared as `attr.output` so cuda_toolkit_info can address each
        # file by label and get the actual generated File (with a
        # bazel-out path), instead of a non-existent source label.
        "ptxas": attr.output(mandatory = True),
        "src_files": attr.label(
            mandatory = True,
            doc = "Filegroup providing every file under the source CUDA toolkit root.",
        ),
        "wrapper": attr.label(
            allow_single_file = True,
            doc = "The wrapper script copied into bin/{ptxas,fatbinary,nvlink,bin2c}. " +
                  "If unset, the intercepted bin/ entries become direct symlinks " +
                  "to the real binaries from src_files.",
        ),
    },
)
