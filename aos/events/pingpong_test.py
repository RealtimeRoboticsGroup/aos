import sys
from typing import Union

import numpy as np
import numpy.typing as npt
from absl.testing import absltest
from typing_extensions import Self

import aos.events.util as util
import ping_fbs_py.aos.examples.Ping as PingFbs
import pong_fbs_py.aos.examples.Pong as PongFbs
from aos.events.event_loop_runtime import EventLoopRuntime
from aos.events.ping import Ping
from aos.events.pong import pong
from aos.events.simulated_event_loop import SimulatedEventLoopFactory


class Int64Counter:
    """Simple integer counter.

    This encapsulates an integer value. Since integers are immutable in Python, they
    can't be modified in place. This makes them inefficient and hard to share.

    This class represents the integer as a numpy int64 array of unit length, which can
    be modified in place.
    """

    def __init__(self, value: int = 0) -> None:
        self._count: npt.NDArray[np.int64] = np.array([value], dtype=np.int64)

    def __iadd__(self, other: Union[int, "Int64Counter"]) -> Self:
        if isinstance(other, int):
            self._count[0] += other
            return self
        elif isinstance(other, Int64Counter):
            self._count[0] += other._count[0]
            return self
        return NotImplemented

    def count(self) -> np.int64:
        return self._count[0]


class PingPongTest(absltest.TestCase):

    def setUp(self):
        """Sets up for a ping-pong test.

        Reads the AOS configuration, instantiates a simulated event loop factory, and
        spawns Ping and Pong tasks.
        """
        self._config = util.Configuration(
            "aos/aos/events/pingpong_config.bfbs")
        self._factory = SimulatedEventLoopFactory(self._config)

        ping_runtime = self._factory.make_runtime("ping", "")
        ping = Ping()
        ping_runtime.spawn_tasks(ping.tasks(ping_runtime, 10_000_000))

        pong_runtime = self._factory.make_runtime("pong", "")
        pong_runtime.spawn_task(pong(pong_runtime))

    def tearDown(self):
        """Cleans up resources allocated in the simulated event loop factory."""
        self._factory.shut_down()

    def _run_for(self, duration_ns: int) -> None:
        """Advances simulated time by `duration_ns` nanoseconds."""
        self._factory.run_for(duration_ns)

    def test_starts(self):
        """Tests that Ping and Pong tasks start up and run successfully."""
        self._run_for(1_000_000_000)

    def test_always_replies(self):
        """Tests that correct number of Ping and Pong messages are exchanged."""

        async def count_pings(runtime: EventLoopRuntime,
                              ping_counter: Int64Counter):
            fetcher = runtime.make_fetcher("/test", PingFbs.Ping)
            await runtime.on_run()
            while True:
                await fetcher.next()
                ping_counter += 1

        async def count_pongs(runtime: EventLoopRuntime,
                              pong_counter: Int64Counter):
            fetcher = runtime.make_fetcher("/test", PongFbs.Pong)
            await runtime.on_run()
            while True:
                await fetcher.next()
                pong_counter += 1

        runtime = self._factory.make_runtime("test", "")
        ping_counter = Int64Counter()
        runtime.spawn_task(count_pings(runtime, ping_counter))
        pong_counter = Int64Counter()
        runtime.spawn_task(count_pongs(runtime, pong_counter))

        self._run_for(10_005_000_000)
        self.assertEqual(ping_counter.count(), 1001)
        self.assertEqual(pong_counter.count(), 1001)


if __name__ == "__main__":
    util.init(sys.argv)
    absltest.main()
