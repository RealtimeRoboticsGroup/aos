from collections.abc import Callable
from typing import Any, Optional, Type, List

from aos.events.event_loop_c import ffi, lib
from aos.events.event_loop import EventLoop, ExceptionPassthroughReceiver, ExitHandle, Error, InternalProxy
from aos.events.event_loop_runtime import EventLoopRuntime
from aos.events.util import Configuration


class _ShmEventLoop(EventLoop):

    def __init__(self, c_event_loop):
        super().__init__(c_event_loop)
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


class ShmEventLoop:

    def __init__(self, config: Configuration) -> None:
        self._config: Configuration = config
        self._event_loop = None

    def __enter__(self) -> _ShmEventLoop:
        c_event_loop = lib.aos_shm_event_loop_create(self._config.get_config())
        self._event_loop = _ShmEventLoop(c_event_loop)
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
