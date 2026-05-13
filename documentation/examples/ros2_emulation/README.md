This folder contains some examples of emulating ROS2 APIs on top of the AOS equivalents, in Python and C++. This is accomplished with "facade" classes which implement the most common parts of the ROS2 API using AOS.

This may be useful when migrating from ROS2 to AOS, but it will never be complete. Implementations of additional APIs are welcome!

The recommended way to migrate with this infrastructure is:

1. Run `convert_msg.py` and review the output. This tool is very hacky and will fail and misinterpret many .msg files, so you need to review the output. The outputs are not marked as generated because they are intended to be used as source files after the migration is complete. You may want to maintain the .msg files as the source of truth during a migration with concurrent development, and run `convert_msg.py` again periodically.
2. Update everything. Python `import` statements won't have to change, but C++ `#include`s will. All `main` functions (in both languages) and corresponding ROS2 `Node` instantiations will need to change. All `BUILD` files will need to be updated to use the targets in this folder and the outputs from `convert_msg.py`.
3. Test everything and make changes as needed. If you have code using any ROS2 APIs not implemented by the code in this folder, either add those APIs or modify the code. Modifying code to use APIs supported in both places can be a good strategy, because those changes can be merged incrementally.
4. Clean up and remove all ROS2 components.
5. Gradually migrate away from the classes in this folder. They expose the AOS `EventLoop` directly, so once that instance is the only usage the classes in this folder can be replaced with the `EventLoop` itself.

# Tips and tricks

You can create as many instances of the facade classes as you want. This is helpful for converting individual components to the AOS APIs directly without converting all of their dependencies first.

The facade classes ignore the ROS2 qos parameters. AOS provides similar functionality (applied to all processes) in the config.

The C++ generated code handles list of nested tables differently. The flatbuffers based API wraps the nested tables in a `std::unique_ptr`, unlike the ROS2 equivalent. You can either create a table using `std::make_unique` and fill it out, or create the table in-place (like with ROS2) and call `std::make_unique` at the `push_back` callsite.

ROS2 and AOS use the term "node" differently. It may be helpful to refer to the concepts as "computer" (AOS node, ROS2 doesn't have a name) and "process" (ROS2 node, AOS event loop). AOS uses the term "application" to refer specifically to a process which uses an event loop, although the generic "event loop" is used too.

## Channels sent from multiple AOS nodes

Unlike ROS2 topics, AOS channels may only be sent from a single AOS node. AOS handles this situation with separate channels for each node. Node-scoped maps in the AOS config may be used to map senders to the correct channel, but any fetchers or watchers will need to understand they are interacting with multiple underlying channels.

Note that messages may be received in different orders between the AOS channels, which is not possible with the ROS2 single-topic equivalent.

## Channels sent and received from the same process

ROS2 allows sending and receiving on the same topic in a single process, but the behavior appears to be RMW-specific and incompletely implemented by some common RMW (https://github.com/ros2/ros2/issues/1095 and https://answers.ros.org/question/357792/).

AOS handles this situation with separate channels for each process. Application-scoped maps (which apply to a single process) in the AOS config may be used to map senders to the correct channel, but any fetchers or watchers will need to understand they are interacting with multiple underlying channels.

Note that messages may be received in different orders between the AOS channels, which is not possible with the ROS2 single-topic equivalent.

## Handle object lifetimes

ROS2 and AOS both expose handle objects for senders (publishers) and timers. ROS2 also exposes handle objects for watchers (subscribers). ROS2 hands ownership of each of these objects to the caller, while AOS only hands ownership of senders and watchers/timers live as long as the `EventLoop`. Most ROS2 code destroys all handles at the same time as the ROS2 `Node` so this is equivalent, and the facades provide trivial non-owning handles for API compatibility.

ROS2 Python code often contains explicit destruction logic to avoid reference cycles extending lifetimes too long. This can all be ignored with AOS, the AOS Python API has a simplified lifetime/ownership model where all C++ ownership is tied to a context manager so its lifetime is explicit and Python cycles have no consequence.

## Thread scheduling

The AOS `EventLoop` scheduling APIs (`SetRuntimeRealtimePriority` and `SetRuntimeAffinity`) replace directly setting priorities with ROS2, and integrate with additional AOS functionality not present in ROS2. Any code to configure the scheduling of ROS2 helper threads should be discarded; AOS does not use helper threads.

# Design notes

The Python facades use different type names because these rarely appear in Python code using ROS2, and using different names makes it easier to follow. The C++ facades use the same names because they show up frequently in C++ code, so changing all of them would make migration significantly harder.
