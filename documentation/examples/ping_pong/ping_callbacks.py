import flatbuffers
from absl import app
from absl import flags
from absl import logging

import aos.events.util as util
from ping_fbs_py.aos.examples.Ping import PingT
from pong_fbs_py.aos.examples.Pong import PongT
from aos.events.shm_event_loop import ShmEventLoop

FLAGS = flags.FLAGS

flags.DEFINE_integer("sleep_ns", 10_000_000, "Nanoseconds between pings.")


class Ping:

    def __init__(self, event_loop):
        self._event_loop = event_loop
        self._sender = self._event_loop.make_sender(PingT, "/test")
        self._timer_handle = self._event_loop.add_timer(self.send_ping)
        self._timer_handle.set_name("ping")
        self._event_loop.make_watcher(PongT, "/test", self.handle_pong)
        self._event_loop.on_run(lambda: self._timer_handle.schedule_ns(
            self._event_loop.monotonic_now_ns(), FLAGS.sleep_ns))
        self._event_loop.set_runtime_realtime_priority(5)

        self._last_pong_value = 0
        self._count = 0

        logging.info("Now running ping application.")

    def send_ping(self):
        if self._last_pong_value != self._count:
            logging.vlog(1, "Did not receive response to %d within %d ns",
                         self._count, FLAGS.sleep_ns)
        self._count += 1

        ping = PingT()
        ping.value = self._count
        ping.send_time = self._event_loop.monotonic_now_ns()
        self._sender.send(ping)
        logging.vlog(2, "Sending ping")

    def handle_pong(self, pong):
        monotonic_send_time = pong.initial_send_time
        monotonic_now = self._event_loop.monotonic_now_ns()
        round_trip_time = monotonic_now - monotonic_send_time
        if self._last_pong_value + 1 != pong.value:
            logging.vlog(1, "Unexpected pong value, wanted %d, got %d",
                         self._last_pong_value + 1, pong.value)
        if pong.value == self._count:
            logging.vlog(1, "Elapsed time %d ns %r", round_trip_time, pong)
        else:
            logging.warning(
                "Unexpected pong response, got %r expected %d, elapsed time %d ns",
                pong, self._count, round_trip_time)
        self._last_pong_value = pong.value


def main(argv):
    util.init(argv)
    config = util.Configuration(
        "aos/documentation/examples/ping_pong/pingpong_config.bfbs")
    with ShmEventLoop(config) as shm_event_loop:
        ping = Ping(shm_event_loop)
        shm_event_loop.run()


if __name__ == "__main__":
    app.run(main)
