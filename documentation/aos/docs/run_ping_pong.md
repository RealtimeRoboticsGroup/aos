# How to run ping & pong

Running ping<->pong is a nice way to test that you can run some basic code. This example shows how messaging can work between two processes.

## Set up real-time niceties:

You only have to do this once. This allows the processes to run in a "sufficiently realtime" manner.

  1. Add the following lines to `/etc/security/limits.d/rt.conf`, replacing "USERNAME" with the username you're running under.  You'll probably need to do this as root, e.g., `sudo nano /etc/security/limits.d/rt.conf`.
```
USERNAME - nice -20
USERNAME - rtprio 95
USERNAME - memlock unlimited
```

  2. Reboot your machine to pick up the changes.

## Compile and run the code
  1. Compile the code for ping and pong, as well as aos_dump for looking at the messages.  We'll assume throughout that you're running from the top level directory of aos.
  ```
  bazel build -c opt //aos/events:ping //aos/events:pong //aos:aos_dump
  ```

  2. In 2 separate windows, run the ping and pong commands using the `pingpong_config.json` config file:
    1. `bazel-bin/aos/events/ping --config bazel-bin/aos/events/pingpong_config.json`
    2. `bazel-bin/aos/events/pong --config bazel-bin/aos/events/pingpong_config.json`

  3. In a third window, explore the message stream using `aos_dump`.  Some things you can do:
    1. List the channels:
       `bazel-bin/aos/aos_dump --config bazel-bin/aos/events/pingpong_config.json`
    2. Listen to a specific topic on a channel-- copy one of the channels listed in the first step and put it at the end of the aos_dump command (e.g., "/test aos.examples.Ping"):
       `bazel-bin/aos/aos_dump --config bazel-bin/aos/events/pingpong_config.json /test aos.examples.Ping`
    3. Listen to multiple topics on a channel (e.g., all the topics published on "/test"):
       `bazel-bin/aos/aos_dump --config bazel-bin/aos/events/pingpong_config.json /test`


NOTE: To make life easier, you can alias `aos_dump` to include the path and the config file (you may want to specify the full path to the aos_dump executable so it can be run from anywhere)
```
alias aos_dump='bazel-bin/aos/aos_dump --config bazel-bin/aos/events/pingpong_config.json'
```

## Logging

In addition to running ping and pong, this is a good example to explore event logging.

  1. Start by compiling the code:
     ```
     bazel build -c opt //aos/events/logging:logger_main //aos/events/logging:log_cat
     ```

  2. Create a folder for the log files, e.g., 
     ```
     mkdir /tmp/log_folder
     ```

  3. Run the logger to log the data:
     ```  
     bazel-bin/aos/events/logging/logger_main --config bazel-bin/aos/events/pingpong_config.json --logging_folder /tmp/log_folder/
     ```
  
     A log folder should be created in /tmp/log_folder, with a name something like `fbs_log_-<number>_<date and time>`.

     If you're running ping and pong at the same time, you should be able to watch the log file grow in size as events are being logged.  For example, running `ls -lh /tmp/log_folder/*` will list the files and their sizes.  Doing this periodically (e.g., every 15-30 seconds) should show the new log file growing in size.

   4. View the contents of the log file using `log_cat`. Run the binary pointed at the recorded event log.:
      ```
      bazel-bin/aos/events/logging/log_cat /tmp/log_folder/fbs_log_-<number>_<date and time>
      ```
      You will a list of all messages, in the format
      ```
      <wallclock time> <second since start> <channel> <message type> <message as json>
      ```

## EXERCISE:
   1. Modify code in ping and pong to see a difference in their behavior, e.g., increment the counter by 2's instead of 1

## Using starterd

We have a python script which spins up a copy of starter with ping, pong, and the other CLI tools ready for use.  To use it, run it from Bazel:

```
$ bazel run -c opt //aos/starter:starter_demo
INFO: Analyzed target //aos/starter:starter_demo (41 packages loaded, 1059 targets configured).
INFO: Found 1 target...
Target //aos/starter:starter_demo up-to-date:
  bazel-bin/aos/starter/starter_demo
INFO: Elapsed time: 3.441s, Critical Path: 0.09s
INFO: 1 process: 1 internal.
INFO: Build completed successfully, 1 total action
INFO: Running command line: bazel-bin/aos/starter/starter_demo aos/starter/starterd 'aos/events/pingpong_config.bfbs aos/events/pingpong_config.stripped.json' aos/events/ping aos/events/pong aos/starter/aos_starter aos/aos_dump aos/events/logging/logger_main aos/events/aos_timing_report_streamer
Running starter from /tmp/tmp69_sqrct


To run aos_starter, do:
cd /tmp/tmp69_sqrct
./aos_starter


I0220 19:05:06.062071 3885753 starterd_lib.cc:132] Starting to initialize shared memory.
```

You can then cd to that directory and play with it.

If you want to instead build a .tar of the applications to run somewhere else, you can build a package with the binaries.

```
$ bazel build -c opt //aos/starter:ping_pong_demo
INFO: Analyzed target //aos/starter:ping_pong_demo (16 packages loaded, 33761 targets configured).
INFO: Found 1 target...
Target //aos/starter:ping_pong_demo up-to-date:
  bazel-bin/aos/starter/ping_pong_demo.tar
INFO: Elapsed time: 7.248s, Critical Path: 4.46s
INFO: 44 processes: 29 disk cache hit, 2 internal, 13 linux-sandbox.
INFO: Build completed successfully, 44 total actions
```

To run everything by hand, just run `./starterd` after extracting, and everything will start up automatically.

### Benchmarking

`ping` and `pong` serve as a good pair of latency benchmarking applications for doing initial checking of the performance of a system.
By default, they run with a RT priority of 5, and they can also be configured to forward across the network with a custom `aos_config.bfbs`.

`aos_timing_report_streamer` prints out timing reports in a nice, human parsable format.
`Control-C` stops it and prints overall aggregated statistics.
The max wakeup (and handler) latency can be used to measure the jitter of your system.
For example, on a `12th Gen Intel(R) Core(TM) i7-12700K`, for 1 second of execution time, I get:

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

Here, we can see, for the 1 `aos.timing.Report` was sent (1 second), 100 `aos.examples.Pong` messages were received, 100 `aos.examples.Ping` messages were sent, and the ping timer triggered 100 times.
The ping timer callback took on average 26 uS between when it was scheduled, and when it actually triggered, with a max of 100 uS.
The pong callback had a mean of 36 uS between when the `aos.examples.Pong` message was published, and when the watcher callback started, and took on average 0.7 uS to execute.

Not bad!
