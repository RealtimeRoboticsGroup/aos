def format_multirun(name, **kwargs):
    filegroup_args = {}
    for attr in ["visibility", "tags", "testonly", "target_compatible_with"]:
        if attr in kwargs:
            filegroup_args[attr] = kwargs[attr]
    native.filegroup(name = name, **filegroup_args)
