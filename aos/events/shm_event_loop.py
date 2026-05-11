from collections.abc import Callable
from typing import Any, Optional, Type

from aos.events.event_loop_c import lib
from aos.events.event_loop_runtime import EventLoopRuntime
from aos.events.util import Configuration


class ShmEventLoop:

    def __init__(self, config: Configuration) -> None:
        self._config: Configuration = config
        self._c_event_loop = None

    def __enter__(self) -> "ShmEventLoop":
        self._c_event_loop = lib.aos_shm_event_loop_create(
            self._config.get_config())
        return self

    def __exit__(
        self,
        exc_type: Optional[Type[BaseException]],
        exc_value: Optional[BaseException],
        traceback: Optional[Any],
    ):
        lib.aos_event_loop_destroy(self._c_event_loop)
        self._c_event_loop = None

    def run_with(self, task: Callable[[EventLoopRuntime], None]) -> None:
        if not self._c_event_loop:
            raise RuntimeError("Use ShmEventLoop as a context manager")
        with EventLoopRuntime(self._c_event_loop) as runtime:
            task(runtime)
            lib.aos_error_destroy(
                lib.aos_shm_event_loop_run(self._c_event_loop))
