# Starterd

Starterd is the system for launching a complete system. It defines a system of applications across a complex distributed system and starts those applications and relavent all tools.

## Using starterd

We have a python script which spins up a copy of starter with ping, pong, and the other CLI tools ready for use. To use it, run it from Bazel:

```
$ bazel run -c opt //documentation/examples/starterd:starter_demo
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

## Starting Remotely

If you want to instead build a .tar of the applications to run somewhere else, you can build a package with the binaries.

```
$ bazel build -c opt //documentation/examples/starterd:ping_pong_demo
INFO: Analyzed target //aos/starter:ping_pong_demo (16 packages loaded, 33761 targets configured).
INFO: Found 1 target...
Target //documentation/examples/ping_pong:ping_pong_demo up-to-date:
  bazel-bin/documentation/examples/ping_pong/ping_pong_demo.tar
INFO: Elapsed time: 7.248s, Critical Path: 4.46s
INFO: 44 processes: 29 disk cache hit, 2 internal, 13 linux-sandbox.
INFO: Build completed successfully, 44 total actions
```

To run everything by hand, just run `./starterd` after extracting, and everything will start up automatically. To run each application manually, in there own terminal run `./ping` and `./pong`. Then to view the messages simply run `./aos_dump --config pingpong_config.json`
