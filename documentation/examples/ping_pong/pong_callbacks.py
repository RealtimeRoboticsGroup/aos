import flatbuffers
from absl import app
from absl import flags
from absl import logging

import aos.events.util as util
from ping_fbs_py.aos.examples.Ping import PingT
from pong_fbs_py.aos.examples.Pong import PongT
from aos.events.shm_event_loop import ShmEventLoop

FLAGS = flags.FLAGS

# TODO(Brian): Add fetch mode like pong_lib.cc has.


class Pong:

    def __init__(self, event_loop):
        self._event_loop = event_loop
        self._sender = self._event_loop.make_sender(PongT, "/test")
        self._event_loop.make_watcher(PingT, "/test", self.handle_ping)
        self._event_loop.set_runtime_realtime_priority(5)

        self._last_value = 0
        self._last_send_time = 0

    def handle_ping(self, ping):
        if self._last_value == ping.value:
            logging.vlog(1, "Duplicate ping value at %d time difference %d ns",
                         self._last_value,
                         ping.send_time - self._last_send_time)
        if self._last_send_time == 0:
            logging.info("Now running pong application.")

        self._last_value = ping.value
        self._last_send_time = ping.send_time
        pong = PongT()
        pong.value = ping.value
        pong.initial_send_time = ping.send_time
        self._sender.send(pong)


def main(argv):
    util.init(argv)
    config = util.ConfigurationBuffer(
        "aos/documentation/examples/ping_pong/pingpong_config.bfbs")
    with ShmEventLoop(config) as shm_event_loop:
        pong = Pong(shm_event_loop)
        shm_event_loop.run()


if __name__ == "__main__":
    app.run(main)
