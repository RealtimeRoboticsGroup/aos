import flatbuffers
from absl import app
from absl import flags
from absl import logging

import aos.events.util as util
import ping_fbs_py.aos.examples.Ping as PingFbs
import pong_fbs_py.aos.examples.Pong as PongFbs
from aos.events.shm_event_loop import ShmEventLoop

FLAGS = flags.FLAGS

flags.DEFINE_integer("sleep_ns", 10_000_000, "Nanoseconds between pings.")


class Ping:

    def __init__(self):
        self._value = 0

    def tasks(self, runtime, sleep_ns):
        return [
            self.__send_ping(runtime, sleep_ns),
            self.__handle_pong(runtime),
            self.__another_handle_pong(runtime),
        ]

    async def __send_ping(self, runtime, sleep_ns):
        sender = runtime.make_sender("/test", PingFbs.Ping)
        timer = runtime.add_timer()
        await runtime.on_run()
        timer.schedule(runtime.monotonic_now(), sleep_ns)
        while True:
            await timer.tick()
            self._value += 1
            fbb = flatbuffers.Builder()
            PingFbs.Start(fbb)
            PingFbs.AddValue(fbb, self._value)
            PingFbs.AddSendTime(fbb, runtime.monotonic_now())
            fbb.Finish(PingFbs.End(fbb))
            sender.send(fbb.Output())

    async def __handle_pong(self, runtime):
        fetcher = runtime.make_fetcher("/test", PongFbs.Pong)
        await runtime.on_run()
        while True:
            pong = await fetcher.next()
            now = runtime.monotonic_now()
            round_trip_time = now - pong.InitialSendTime()
            logging.vlog(1, f"Round trip time: {round_trip_time}ns")

    async def __another_handle_pong(self, runtime):
        fetcher = runtime.make_fetcher("/test", PongFbs.Pong)
        await runtime.on_run()
        while True:
            await fetcher.next()
            logging.vlog(1, "Got pong in other task")


def run_ping(runtime):
    # TODO(Sanjay): Uncomment the following line once the C API is implemented for it.
    # runtime.set_realtime_priority(5)
    ping = Ping()
    runtime.spawn_tasks(ping.tasks(runtime, FLAGS.sleep_ns))


def main(argv):
    util.init(argv)
    config = util.Configuration("aos/aos/events/pingpong_config.bfbs")
    with ShmEventLoop(config) as shm_event_loop:
        shm_event_loop.run_with(run_ping)


if __name__ == "__main__":
    app.run(main)
