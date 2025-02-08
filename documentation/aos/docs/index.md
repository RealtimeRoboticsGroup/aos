# Introduction

AOS ("Autonomous Operating System") is an open source event-loop based framework for
realtime robotic systems. It is meant to serve the same role as
[ROS](https://www.ros.org/) in a robotic system (including the fact that, like
ROS, it is not actually an Operating System). However, the design priorities of
AOS are somewhat different from those of both ROS and ROS2. See the [Design of
AOS](design) for more detail.

# Contributing

See [here](./README.md#Contributing)

# Documentation

## Quickstart
If you want to start by running something locally, try [running ping/pong](run_ping_pong.md). This includes an example of logging.

If curious how this example was built, see [building ping/pong](build_ping_pong.md).

## Design of AOS

A 1-hour long introduction to AOS, from August 2020, can be found
[here][youtube]; the corresponding
presentation is available
[here][slidedeck].

The information in that presentation is still essentially correct, with the main
changes since 2020 having been improvements to general stability and increased
feature support for things outside of the core functionality discussed there.

[youtube]: https://www.youtube.com/watch?v=vE1Ll5KDNQU
[slidedeck]: https://docs.google.com/presentation/d/1G4J2a47f3v1m1Wq2ZO5pEADg6yJirEh07INRywv-4BQ/edit?usp=sharing

## Reference
Exhaustive [reference](reference.md).
