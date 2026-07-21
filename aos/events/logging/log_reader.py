from typing import Any, Optional, Type

from aos.events.event_loop_c import lib, ffi
from aos.events.util import ConfigurationBuffer


class LogReader:

    def __init__(self, log_path: str) -> None:
        self._log_path = log_path
        self._c_log_reader = ffi.NULL
        self._log_reader = None

    def __enter__(self) -> 'LogReader':
        argv = [
            ffi.new("char[]", b"logreader"),
            ffi.new("char[]", self._log_path.encode("utf-8"))
        ]
        argv_array = ffi.new("char*[]", argv)
        self._c_log_reader = lib.aos_log_reader_create_from_argv(2, argv_array)
        if self._c_log_reader == ffi.NULL:
            raise RuntimeError(
                f"Failed to create LogReader for {self._log_path}")
        self._log_reader = _LogReader(self._c_log_reader)
        return self._log_reader

    def __exit__(
        self,
        exc_type: Optional[Type[BaseException]],
        exc_value: Optional[BaseException],
        traceback: Optional[Any],
    ) -> bool:
        if self._log_reader is not None:
            self._log_reader.deregister()
        if self._c_log_reader != ffi.NULL:
            lib.aos_log_reader_destroy(self._c_log_reader)
            self._c_log_reader = ffi.NULL
        return False


class _LogReader:

    def __init__(self, c_log_reader) -> None:
        self._c_log_reader = c_log_reader

    def configuration_buffer(self) -> ConfigurationBuffer:
        c_buf = lib.aos_log_reader_configuration_buffer(self._c_log_reader)
        c_data = lib.aos_configuration_buffer_get_data(c_buf)
        size = lib.aos_configuration_buffer_get_size(c_buf)
        data = bytes(ffi.buffer(c_data, size))
        lib.aos_configuration_buffer_destroy(c_buf)
        return ConfigurationBuffer.from_bytes(data)

    def register(self, factory) -> None:
        lib.aos_log_reader_register(self._c_log_reader, factory._c_factory)

    def deregister(self) -> None:
        '''This is called as part of exiting the context manager, so it is
        usually unnecessary to call separately.

        Calling this when not registered does nothing.'''
        lib.aos_log_reader_deregister(self._c_log_reader)
