import flatbuffers
from absl import app
from absl import logging

import aos.events.util as util
import ping_fbs_py.aos.examples.Ping as PingFbs
import pong_fbs_py.aos.examples.Pong as PongFbs
from aos.events.shm_event_loop import ShmEventLoop


async def pong(runtime):
    fetcher = runtime.make_fetcher("/test", PingFbs.Ping)
    sender = runtime.make_sender("/test", PongFbs.Pong)
    await runtime.on_run()
    while True:
        ping = await fetcher.next()
        fetched, fetched_ping = fetcher.fetch()
        assert fetched
        assert ping.Value() == fetched_ping.Value()
        logging.vlog(1, f"Received Ping({ping.Value()})")
        fbb = flatbuffers.Builder()
        PongFbs.Start(fbb)
        PongFbs.AddValue(fbb, ping.Value())
        PongFbs.AddInitialSendTime(fbb, ping.SendTime())
        fbb.Finish(PongFbs.End(fbb))
        sender.send(fbb.Output())


def run_pong(runtime):
    # TODO(Sanjay): Uncomment the following line once the C API is implemented for it.
    # runtime.set_realtime_priority(5)
    runtime.spawn_task(pong(runtime))


def main(argv):
    util.init(argv)
    config = util.Configuration("aos/aos/events/pingpong_config.bfbs")
    with ShmEventLoop(config) as shm_event_loop:
        shm_event_loop.run_with(run_pong)


if __name__ == "__main__":
    app.run(main)
