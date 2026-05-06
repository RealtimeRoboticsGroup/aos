import sys
import contextlib

from absl.testing import absltest

import aos.events.util as util
from ping_fbs_py.aos.examples.Ping import PingT
from pong_fbs_py.aos.examples.Pong import PongT
from documentation.examples.ping_pong.ping_callbacks import Ping
from documentation.examples.ping_pong.pong_callbacks import Pong
from aos.events.simulated_event_loop import SimulatedEventLoopFactory


class PingPongTests(absltest.TestCase):

    def setUp(self):
        self._config = util.Configuration(
            "aos/documentation/examples/ping_pong/pingpong_config.bfbs")
        with contextlib.ExitStack() as stack:
            self._factory = stack.enter_context(
                SimulatedEventLoopFactory(self._config))
            self.addCleanup(stack.pop_all().close)

        self._ping = Ping(self._factory.make_event_loop("ping", ""))
        self._pong = Pong(self._factory.make_event_loop("pong", ""))

    def test_starts(self):
        """Tests that we can startup at all.

        This confirms that the channels are all in the config."""
        self._factory.run_for_ns(10_000_000_000)

    def test_always_replies(self):
        """Tests that the number of pong messages matches the number of ping messages."""
        test_event_loop = self._factory.make_event_loop("test", "")

        ping_count = 0
        pong_count = 0

        # Confirm that the ping value matches.
        def handle_ping(ping):
            nonlocal ping_count
            self.assertEqual(ping.value, ping_count + 1)
            ping_count += 1

        test_event_loop.make_watcher(PingT, "/test", handle_ping)

        # Confirm that the ping and pong counts both match, and the value also
        # matches.
        def handle_pong(pong):
            nonlocal pong_count
            self.assertEqual(pong.value, pong_count + 1)
            pong_count += 1
            self.assertEqual(ping_count, pong_count)

        test_event_loop.make_watcher(PongT, "/test", handle_pong)

        self._factory.run_for_ns(10_000_000_000)

        # We run at t=0 and t=10 seconds, which means we run 1 extra time.
        self.assertEqual(ping_count, 1001)


class MultiNodePingPongTest(absltest.TestCase):
    """Multi-node ping pong test.

    This test carefully mirrors the structure of the single node test above to
    help highlight the similarities and differences."""

    def setUp(self):
        self._config = util.Configuration(
            "aos/documentation/examples/ping_pong/multinode_pingpong_test_split_config.bfbs"
        )
        with contextlib.ExitStack() as stack:
            self._factory = stack.enter_context(
                SimulatedEventLoopFactory(self._config))
            self.addCleanup(stack.pop_all().close)

        self._ping = Ping(self._factory.make_event_loop("ping", "pi1"))
        self._pong = Pong(self._factory.make_event_loop("pong", "pi2"))

    def test_always_replies(self):
        """Tests that the number of pong messages matches the number of ping messages (on both nodes this time)."""
        pi1_test_event_loop = self._factory.make_event_loop("test", "pi1")
        pi2_test_event_loop = self._factory.make_event_loop("test", "pi2")

        pi1_ping_count = 0
        pi1_pong_count = 0
        pi2_ping_count = 0
        pi2_pong_count = 0

        # Confirm that the ping value matches on both nodes.
        def pi1_handle_ping(ping):
            nonlocal pi1_ping_count
            self.assertEqual(ping.value, pi1_ping_count + 1)
            pi1_ping_count += 1

        def pi2_handle_ping(ping):
            nonlocal pi2_ping_count
            self.assertEqual(ping.value, pi2_ping_count + 1)
            pi2_ping_count += 1

        pi1_test_event_loop.make_watcher(PingT, "/test", pi1_handle_ping)
        pi2_test_event_loop.make_watcher(PingT, "/test", pi2_handle_ping)

        # Confirm that the ping and pong counts both match, and the value also
        # matches.
        def pi1_handle_pong(pong):
            nonlocal pi1_pong_count
            self.assertEqual(pong.value, pi1_pong_count + 1)
            pi1_pong_count += 1
            self.assertEqual(pi1_ping_count, pi1_pong_count)

        def pi2_handle_pong(pong):
            nonlocal pi2_pong_count
            self.assertEqual(pong.value, pi2_pong_count + 1)
            pi2_pong_count += 1
            self.assertEqual(pi2_ping_count, pi2_pong_count)

        pi1_test_event_loop.make_watcher(PongT, "/test", pi1_handle_pong)
        pi2_test_event_loop.make_watcher(PongT, "/test", pi2_handle_pong)

        # Since forwarding takes "time", we need to run a bit longer to let pong
        # finish the last cycle.
        self._factory.run_for_ns(10_005_000_000)

        # We run at t=0 and t=10 seconds, which means we run 1 extra time.
        self.assertEqual(pi1_ping_count, 1001)
        self.assertEqual(pi2_ping_count, 1001)


if __name__ == "__main__":
    util.init(sys.argv)
    absltest.main()
