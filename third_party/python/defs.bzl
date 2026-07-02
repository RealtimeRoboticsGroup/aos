def _extract_numpy_headers_impl(ctx):
    files = ctx.attr.numpy[DefaultInfo].default_runfiles.files.to_list()
    prefix = ctx.attr.header_prefix

    hdrs = []
    for file in files:
        _, partition, include_file = file.path.partition("/numpy/core/include/numpy/")
        if not partition:
            _, partition, include_file = file.path.partition("/numpy/_core/include/numpy/")
        if partition:
            hdr = ctx.actions.declare_file("%s/numpy/%s" % (prefix, include_file))
            ctx.actions.run(
                inputs = [file],
                outputs = [hdr],
                executable = "cp",
                arguments = [file.path, hdr.path],
            )
            hdrs.append(hdr)

    return [DefaultInfo(files = depset(hdrs))]

extract_numpy_headers = rule(
    implementation = _extract_numpy_headers_impl,
    doc = "Extracts the numpy headers from the corresponding py_library target.",
    attrs = {
        "header_prefix": attr.string(
            mandatory = True,
            doc = "The directory to copy the headers into.",
        ),
        "numpy": attr.label(
            mandatory = True,
            providers = [PyInfo],
            doc = "The label for the numpy py_library target.",
        ),
    },
)
