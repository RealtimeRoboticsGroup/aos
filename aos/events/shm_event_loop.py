from collections.abc import Callable
from typing import Any, Optional, Type, List
import select

from aos.events.event_loop_c import ffi, lib
from aos.events.event_loop import EventLoop, ExceptionPassthroughReceiver, ExitHandle, Error, InternalProxy, _intercept_exception, ExceptionPassthroughInterceptor
from aos.events.event_loop_runtime import EventLoopRuntime
from aos.events.util import ConfigurationBuffer


class _ShmEventLoop(EventLoop):

    def __init__(self, c_event_loop: ffi.CData,
                 config_buffer: ConfigurationBuffer):
        super().__init__(c_event_loop, config_buffer)
        self._exit_handles: list[ExitHandle] = []
        self._proxies: List[InternalProxy] = list()

    def set_name(self, name: str) -> None:
        name_bytes = name.encode('utf-8')
        lib.aos_shm_event_loop_set_name(self._c_event_loop,
                                        ffi.from_buffer(name_bytes),
                                        len(name_bytes))

    def close(self):
        for proxy in self._proxies:
            proxy._event_loop_proxy_target = None
        self._proxies.clear()

        for exit_handle in self._exit_handles:
            exit_handle.close()
        self._exit_handles.clear()

        super().close()

    def _add_proxy(self, target):
        proxy = InternalProxy(target)
        self._proxies.append(proxy)
        return proxy

    def make_exit_handle(self):
        exit_handle = ExitHandle(
            lib.aos_shm_event_loop_make_exit_handle(self._c_event_loop))
        self._exit_handles.append(exit_handle)
        return self._add_proxy(exit_handle)

    def run_with(self, task: Callable[[EventLoopRuntime],
                                      None]) -> Optional[Error]:
        with EventLoopRuntime(self._c_event_loop) as runtime:
            task(runtime)
            return self.run()

    def run(self) -> Optional[Error]:
        with ExceptionPassthroughReceiver(
                self.make_exit_handle()) as exception_receiver:
            error = Error(lib.aos_shm_event_loop_run(self._c_event_loop))
            exception_receiver.maybe_reraise_exceptions(error)
            return error

    def lock_to_thread(self):
        lib.aos_shm_event_loop_lock_to_thread(self._c_event_loop)

    def on_fd_readable(self, fd, callback: Callable[[], None]):
        """Registers a function to be called when the fd is readable.

        Only one function may be registered for each fd, use `on_fd_events` to
        register multiple types."""

        def wrapped_callback(events, callback=callback):
            assert events & (select.EPOLLIN | select.EPOLLPRI)
            callback()

        self.on_fd_events(fd, wrapped_callback)
        try:
            self.set_fd_events(fd, select.EPOLLIN | select.EPOLLPRI)
        except:
            self.delete_fd(fd)
            raise

    def on_fd_error(self, fd, callback: Callable[[], None]):
        """Registers a function to be called when the fd has an error.

        Only one function may be registered for each fd, use `on_fd_events` to
        register multiple types."""

        def wrapped_callback(events, callback=callback):
            assert events & select.EPOLLERR
            callback()

        self.on_fd_events(fd, wrapped_callback)
        try:
            self.set_fd_events(fd, select.EPOLLERR)
        except:
            self.delete_fd(fd)
            raise

    def on_fd_writable(self, fd, callback: Callable[[], None]):
        """Registers a function to be called when the fd is writable.

        Only one function may be registered for each fd, use `on_fd_events` to
        register multiple types."""

        def wrapped_callback(events, callback=callback):
            assert events & select.EPOLLOUT
            callback()

        self.on_fd_events(fd, wrapped_callback)
        try:
            self.set_fd_events(fd, select.EPOLLOUT)
        except:
            self.delete_fd(fd)
            raise

    def on_fd_events(self, fd, callback: Callable[[int], None]):
        """Registers a function to be called when the configured events occur on fd.

        The function is passed an argument containing the events which occurred.
        Configure events to call this function for using `set_fd_events`."""
        callback_handle = ffi.new_handle(callback)
        self._callback_handles.append(callback_handle)
        lib.aos_shm_event_loop_on_fd_events(self._c_event_loop, fd,
                                            _ShmEventLoop._fd_events_callback,
                                            callback_handle)

    def delete_fd(self, fd):
        lib.aos_shm_event_loop_delete_fd(self._c_event_loop, fd)

    def set_fd_events(self, fd, events):
        lib.aos_shm_event_loop_set_fd_events(self._c_event_loop, fd, events)

    @ffi.callback("void(void *, uint32_t)", onerror=_intercept_exception)
    def _fd_events_callback(callback_handle, events):
        with ExceptionPassthroughInterceptor():
            callback = ffi.from_handle(callback_handle)
            callback(events)


class ShmEventLoop:

    def __init__(self, config_buffer: ConfigurationBuffer) -> None:
        self._config_buffer: ConfigurationBuffer = config_buffer
        self._event_loop = None

    def __enter__(self) -> _ShmEventLoop:
        c_event_loop = lib.aos_shm_event_loop_create(
            ffi.cast('aos_configuration_t *', self._config_buffer.c_root()))
        self._event_loop = _ShmEventLoop(c_event_loop, self._config_buffer)
        return self._event_loop

    def __exit__(
        self,
        exc_type: Optional[Type[BaseException]],
        exc_value: Optional[BaseException],
        traceback: Optional[Any],
    ) -> bool:
        self._event_loop.close()
        lib.aos_event_loop_destroy(self._event_loop._c_event_loop)
        self._event_loop = None
        return False
