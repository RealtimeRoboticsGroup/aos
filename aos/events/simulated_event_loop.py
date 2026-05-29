from typing import Any, Optional, Type

from aos.events.event_loop_c import lib, ffi
from aos.events.event_loop_runtime import EventLoopRuntime
from aos.events.event_loop import EventLoop, ExceptionPassthroughReceiver, ExitHandle, Error, InternalProxy
from aos.events.util import ConfigurationBuffer


class _SimulatedEventLoopFactory:

    def __init__(self, c_factory: ffi.CData,
                 config_buffer: ConfigurationBuffer) -> None:
        self._c_factory = c_factory
        self._config_buffer = config_buffer
        self._c_event_loops: list = []
        self._event_loops: list[EventLoop] = []
        self._runtimes: list[EventLoopRuntime] = []
        self._exit_handles: list[ExitHandle] = []
        self._proxies: List[InternalProxy] = list()

    def close(self):
        for proxy in self._proxies:
            proxy._event_loop_proxy_target = None
        self._proxies.clear()

        for exit_handle in self._exit_handles:
            exit_handle.close()
        self._exit_handles.clear()

        for runtime in self._runtimes:
            runtime.close()
        self._runtimes.clear()

        for event_loop in self._event_loops:
            event_loop.close()
        self._event_loops.clear()

        for c_event_loop in self._c_event_loops:
            lib.aos_event_loop_destroy(c_event_loop)

        lib.aos_simulated_event_loop_factory_destroy(self._c_factory)

    def _add_proxy(self, target):
        proxy = InternalProxy(target)
        self._proxies.append(proxy)
        return proxy

    def make_exit_handle(self):
        exit_handle = ExitHandle(
            lib.aos_simulated_event_loop_factory_make_exit_handle(
                self._c_factory))
        self._exit_handles.append(exit_handle)
        return self._add_proxy(exit_handle)

    def make_runtime(self, name: str, node: str) -> EventLoopRuntime:
        c_name = ffi.new("char[]", name.encode("utf-8"))
        c_node = ffi.new("char[]", node.encode("utf-8"))

        c_event_loop = lib.aos_simulated_event_loop_factory_make_event_loop(
            self._c_factory, c_name, c_node)
        self._c_event_loops.append(c_event_loop)

        runtime = EventLoopRuntime(c_event_loop)
        runtime.init()
        self._runtimes.append(runtime)

        return runtime

    def make_event_loop(self, name: str, node: str) -> EventLoop:
        c_name = ffi.new("char[]", name.encode("utf-8"))
        c_node = ffi.new("char[]", node.encode("utf-8"))

        c_event_loop = lib.aos_simulated_event_loop_factory_make_event_loop(
            self._c_factory, c_name, c_node)
        self._c_event_loops.append(c_event_loop)

        event_loop = EventLoop(c_event_loop, self._config_buffer)
        self._event_loops.append(event_loop)
        return self._add_proxy(event_loop)

    def run_for_ns(self, duration_ns: int) -> Optional[Error]:
        with ExceptionPassthroughReceiver(
                self.make_exit_handle()) as exception_receiver:
            error = Error(
                lib.aos_simulated_event_loop_factory_non_fatal_run_for(
                    self._c_factory, duration_ns))
            exception_receiver.maybe_reraise_exceptions(error)
            return error


class SimulatedEventLoopFactory:

    def __init__(self, config_buffer: ConfigurationBuffer) -> None:
        self._factory = None
        self._config_buffer = config_buffer

    def __enter__(self) -> _SimulatedEventLoopFactory:
        c_factory = lib.aos_simulated_event_loop_factory_create(
            self._config_buffer.c_root())
        self._factory = _SimulatedEventLoopFactory(c_factory,
                                                   self._config_buffer)
        return self._factory

    def __exit__(
        self,
        exc_type: Optional[Type[BaseException]],
        exc_value: Optional[BaseException],
        traceback: Optional[Any],
    ) -> bool:
        self._factory.close()
        self._factory = None
        return False
