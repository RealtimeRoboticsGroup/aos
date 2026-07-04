# Benchmarking

`ping` and `pong` serve as a good pair of latency benchmarking applications for doing initial checking of the performance of a system. Here we will learn how to aggragate timing info over execution and analyze the performance of our application.

## Running Timing Reports

By default, they run with a RT priority of 5, and they can also be configured to forward across the network with a custom `aos_config.bfbs`.

`aos_timing_report_streamer` prints out timing reports in a nice, human parsable format. `Control-C` stops it and prints overall aggregated statistics. The max wakeup (and handler) latency can be used to measure the jitter of your system. For example, on a `12th Gen Intel(R) Core(TM) i7-12700K`, for 1 second of execution time, I get:

```
$ ./aos_timing_report_streamer --application ping
ping[3885755] () version: "ping_version" (634527.158294370sec,2025-02-20_19-12-14.089194802):
  Watchers (1):
    Channel Name |              Type | Count |                                       Wakeup Latency |                                  Handler Time
           /test | aos.examples.Pong |   100 | 3.67391e-05 [5.397e-06, 0.000102438] std 2.65956e-05 | 7.9988e-07 [1.28e-07, 1.6e-06] std 3.5626e-07
  Senders (3):
    Channel Name |                      Type | Count |                   Size | Errors
           /test |         aos.examples.Ping |   100 |      32 [32, 32] std 0 |   0, 0
            /aos |         aos.timing.Report |     1 |   632 [632, 632] std 0 |   0, 0
            /aos | aos.logging.LogMessageFbs |     0 | nan [nan, nan] std nan |   0, 0
  Timers (2):
              Name | Count |                                       Wakeup Latency |                                      Handler Time
              ping |   100 | 2.66082e-05 [4.693e-06, 0.000100601] std 2.49382e-05 | 1.15897e-05 [2.809e-06, 2.626e-05] std 5.1843e-06
    timing_reports |     1 |            2.5341e-05 [2.5341e-05, 2.5341e-05] std 0 |            5.515e-06 [5.515e-06, 5.515e-06] std 0
^C
Accumulated timing reports :

ping[3885755] () version: "ping_version" (-9223372036.854775808sec,(unrepresentable realtime -9223372036854775808)):
  Watchers (1):
    Channel Name |              Type | Count |                                       Wakeup Latency |                                  Handler Time
           /test | aos.examples.Pong |   100 | 3.67391e-05 [5.397e-06, 0.000102438] std 2.65956e-05 | 7.9988e-07 [1.28e-07, 1.6e-06] std 3.5626e-07
  Senders (3):
    Channel Name |                      Type | Count |                   Size | Errors
           /test |         aos.examples.Ping |   100 |      32 [32, 32] std 0 |   0, 0
            /aos |         aos.timing.Report |     1 |   632 [632, 632] std 0 |   0, 0
            /aos | aos.logging.LogMessageFbs |     0 | nan [nan, nan] std nan |   0, 0
  Timers (2):
              Name | Count |                                       Wakeup Latency |                                      Handler Time
              ping |   100 | 2.66082e-05 [4.693e-06, 0.000100601] std 2.49382e-05 | 1.15897e-05 [2.809e-06, 2.626e-05] std 5.1843e-06
    timing_reports |     1 |            2.5341e-05 [2.5341e-05, 2.5341e-05] std 0 |            5.515e-06 [5.515e-06, 5.515e-06] std 0
```

Here, we can see, for the 1 `aos.timing.Report` was sent (1 second), 100 `aos.examples.Pong` messages were received, 100 `aos.examples.Ping` messages were sent, and the ping timer triggered 100 times. The ping timer callback took on average 26 uS between when it was scheduled, and when it actually triggered, with a max of 100 uS. The pong callback had a mean of 36 uS between when the `aos.examples.Pong` message was published, and when the watcher callback started, and took on average 0.7 uS to execute.

Not bad!
