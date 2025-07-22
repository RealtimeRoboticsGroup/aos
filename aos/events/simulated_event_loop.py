from aos.events.event_loop_c import lib, ffi
from aos.events.event_loop_runtime import EventLoopRuntime
from aos.events.util import Configuration


class SimulatedEventLoopFactory:

    def __init__(self, config: Configuration) -> None:
        self._config: Configuration = config
        self._c_factory = lib.create_simulated_event_loop_factory(
            self._config.get_config())
        self._runtimes: list[EventLoopRuntime] = []

    def __del__(self) -> None:
        self.shut_down()

    def shut_down(self) -> None:
        for runtime in self._runtimes:
            runtime.close()
            lib.destroy_event_loop(runtime._loop())
        self._runtimes.clear()
        if self._c_factory:
            lib.destroy_simulated_event_loop_factory(self._c_factory)
            self._c_factory = None

    def make_runtime(self, name: str, node: str) -> EventLoopRuntime:
        c_name = ffi.new("char[]", name.encode("utf-8"))
        c_node = ffi.new("char[]", node.encode("utf-8"))
        runtime = EventLoopRuntime(
            self._c_factory.make_event_loop(self._c_factory, c_name, c_node))
        runtime.init()
        self._runtimes.append(runtime)
        return runtime

    def run_for(self, duration_ns: int) -> None:
        self._c_factory.run_for(self._c_factory, duration_ns)
