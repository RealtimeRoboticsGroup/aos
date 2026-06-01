import sys

from absl.testing import absltest

from aos.events import util
from aos.events.shm_event_loop import ShmEventLoop


class ShmEventLoopTest(absltest.TestCase):

    def test_set_name(self):
        with ShmEventLoop(
                util.Configuration(
                    "aos/aos/events/event_loop_py_config.bfbs")) as event_loop:
            self.assertEqual(event_loop.name(), "python3")
            event_loop.set_name("new_name")
            self.assertEqual(event_loop.name(), "new_name")


if __name__ == "__main__":
    util.init(sys.argv)
    absltest.main()
