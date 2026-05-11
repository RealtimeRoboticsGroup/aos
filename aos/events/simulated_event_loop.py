from typing import Any, Optional, Type

from aos.events.event_loop_c import lib, ffi
from aos.events.event_loop_runtime import EventLoopRuntime
from aos.events.util import Configuration


class SimulatedEventLoopFactory:

    def __init__(self, config: Configuration) -> None:
        self._config: Configuration = config
        self._c_factory = None
        self._event_loops: list = []
        self._runtimes: list[EventLoopRuntime] = []

    def __enter__(self) -> "SimulatedEventLoopFactory":
        self._c_factory = lib.aos_simulated_event_loop_factory_create(
            self._config.get_config())
        return self

    def __exit__(
        self,
        exc_type: Optional[Type[BaseException]],
        exc_value: Optional[BaseException],
        traceback: Optional[Any],
    ):
        for runtime in self._runtimes:
            runtime.close()
        self._runtimes.clear()

        for event_loop in self._event_loops:
            lib.aos_event_loop_destroy(event_loop)
        self._event_loops.clear()

        lib.aos_simulated_event_loop_factory_destroy(self._c_factory)
        self._c_factory = None

    def make_runtime(self, name: str, node: str) -> EventLoopRuntime:
        c_name = ffi.new("char[]", name.encode("utf-8"))
        c_node = ffi.new("char[]", node.encode("utf-8"))
        event_loop = lib.aos_simulated_event_loop_factory_make_event_loop(
            self._c_factory, c_name, c_node)
        self._event_loops.append(event_loop)
        runtime = EventLoopRuntime(event_loop)
        runtime.init()
        self._runtimes.append(runtime)
        return runtime

    def run_for(self, duration_ns: int) -> None:
        lib.aos_simulated_event_loop_factory_run_for(self._c_factory,
                                                     duration_ns)
