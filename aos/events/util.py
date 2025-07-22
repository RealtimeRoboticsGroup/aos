from typing import List

from aos.events.event_loop_c import ffi, lib, locate


def init(argv: List[str]) -> None:
    argc = len(argv)
    c_argc = ffi.new("int *", argc)
    c_argv = [ffi.new("char[]", arg.encode("utf-8")) for arg in argv]
    c_argv_array = ffi.new("char *[]", c_argv)
    c_argv_ptr = ffi.new("char ***", c_argv_array)
    lib.init(c_argc, c_argv_ptr)


class Configuration:

    def __init__(self, relative_config_path: str) -> None:
        config_path = locate(relative_config_path)
        c_config_path = ffi.new("char[]", str(config_path).encode("utf-8"))
        self._config = lib.read_configuration_from_file(c_config_path)

    def __del__(self) -> None:
        lib.destroy_configuration(self._config)

    def get_config(self):
        return self._config
