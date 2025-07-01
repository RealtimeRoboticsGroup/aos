# Introduction

**AOS** (Autonomous Operating System) is an open-source, event-loop-based
framework designed for real-time robotic systems. It serves a similar role to ROS
in robotic applications, though it is not an actual operating system. AOS is
built with a focus on modularity, performance, and ease of use, making it
suitable for a wide range of applications in robotics, automation, and embedded
systems.

## Use Cases
- **Robotics and Automation**: AOS is ideal for robotics platforms and automation
  systems that require deterministic timing, low-latency communication, and
  modular configuration.
- **Embedded Systems**: The library supports resource-constrained environments,
  making it suitable for embedded devices and controllers.
- **Distributed Systems**: With built-in support for inter-process communication
  and configuration, AOS enables scalable distributed applications.
- **Testing and Simulation**: AOS includes utilities for testing, simulation, and
  introspection, facilitating robust development and validation workflows.

## Capabilities
- **Real-Time Scheduling**: Provides real-time clock and scheduling utilities for
  precise task management.
- **Inter-Process Communication**: Includes libraries for efficient and reliable
  IPC, supporting both local and networked communication.
- **Configuration Management**: Offers flexible configuration handling using
  FlatBuffers, with tools for validation, merging, and introspection.
- **Logging and Debugging**: Built-in logging, tracing, and debugging utilities
  to aid in development and production monitoring.
- **Utilities and Abstractions**: A rich set of utilities for memory management,
  synchronization (mutexes, conditions), and type traits.
- **Testing Support**: Comprehensive test infrastructure and example tests to
  ensure code reliability and correctness.

AOS is modular, extensible, and designed to be integrated into a wide range of
system architectures, from single-board computers to large-scale distributed
networks.

# Documentation

## Quickstart
If you want to start by running something locally, try
[running ping/pong](run_ping_pong.md). This includes an example of logging.

If curious how this example was built, see
[building ping/pong](build_ping_pong.md).

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
