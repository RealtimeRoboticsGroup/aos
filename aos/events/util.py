from typing import List

from aos.events.event_loop_c import ffi, lib, locate
from aos.flatbuffers import FlatbufferDetachedBuffer
from aos.configuration_fbs_py.aos.Configuration import ConfigurationT, Configuration
import flatbuffers

__all__ = [
    "ConfigurationBuffer",
    "init",
    "locate",
]


def init(argv: List[str]) -> None:
    argc = len(argv)
    c_argc = ffi.new("int *", argc)
    c_argv = [ffi.new("char[]", arg.encode("utf-8")) for arg in argv]
    c_argv_array = ffi.new("char *[]", c_argv)
    c_argv_ptr = ffi.new("char ***", c_argv_array)
    lib.aos_init(c_argc, c_argv_ptr)


class ConfigurationBuffer(FlatbufferDetachedBuffer):

    def __init__(self, config_path: str) -> None:
        c_config_path = ffi.new("char[]", str(config_path).encode("utf-8"))
        c_config_buffer = lib.aos_configuration_buffer_read_from_file(
            c_config_path)
        try:
            c_data = lib.aos_configuration_buffer_get_data(c_config_buffer)
            size = lib.aos_configuration_buffer_get_size(c_config_buffer)
            super().__init__(ffi.buffer(c_data, size))
        finally:
            lib.aos_configuration_buffer_destroy(c_config_buffer)

    def root_fbs(self) -> Configuration:
        return self.c_to_fbs(Configuration, self.c_root())

    def root_fbs_t(self) -> ConfigurationT:
        return ConfigurationT.InitFromObj(self.root_fbs())
