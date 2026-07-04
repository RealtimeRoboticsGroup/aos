# Threading in AOS

This document covers the threading API of the AOS event loop.

## Overview

As a general rule, AOS applications should be single-threaded. This keeps the application code simple and reduces the risk of bugs. Try to avoid multi-threading if at all possible.

However, there are some cases where multi-threading is necessary or extremely useful. In particular when work can be easily parallelized. For example, a message handler may perform some processing on received data. If this processing can be easily parallelized, it may be a good idea to use multiple threads to perform this processing.

If you use threads in your application, make sure to interact with the event loop only from the main thread. That means sending and fetching messages should only be done in the main thread. For example, threads can process a message that the main thread received. Once the threads are done, the main thread can send out another message containing the processed information.

To keep everything simple, it is a good idea to push work to the threads and receive their work back all within a single AOS event.

## Thread Safety

Unless explicitly stated otherwise in their documentation, all event loop API calls are not thread-safe.

## Threading Support

AOS provides a mechanism for configuring threads in an application. This is done in two steps.

### 1. Define Thread Configuration

Thread configurations are defined in the `aos::ThreadConfiguration` flatbuffer table. These are specified in an application's `Application` entry in the AOS configuration. For example, the following configuration defines an application that runs at realtime priority and uses 2 threads that also run at realtime priority.

```json
{
  "applications": [
    {
      "name": "my_application",
      "scheduling_policy": "SCHEDULER_FIFO",
      "priority": 1,
      "threads": [
        {
          "name": "worker1",
          "scheduling_policy": "SCHEDULER_FIFO",
          "priority": 1
        },
        {
          "name": "worker2",
          "scheduling_policy": "SCHEDULER_FIFO",
          "priority": 1
        }
      ]
    }
  ]
}
```

### 2. Create or Ignore Thread

For each thread specified in the configuration, you must either ignore it or create it at runtime. If a thread is neither ignored nor created, then the event loop's `Run()` call will block until a timeout is reached, then crash. The timeout is configurable via the `--thread_configuration_timeout_seconds` flag.

#### Ignoring a Thread

Ignoring a thread is necessary if you do not want to use it. This can be useful, for example, when you want to dynamically configure the application to not use any threads at runtime. It can be easier to debug a single-threaded application, for example.

Calling `IgnoreThread` must be done before the event loop starts running.

```cpp
aos::EventLoop *event_loop = ...;
event_loop->IgnoreThread("worker1");
event_loop->IgnoreThread("worker2");

// Run the event loop in single-threaded mode.
...
```

#### Creating a Thread

If you want to use a thread that is specified in the configuration, you must create it yourself and then configure it via the `ConfigureThreadAndWaitForRun` method. The thread will have the scheduling policy, priority, and CPU affinity specified in the configuration.

The thread must be created before the event loop starts running.

A thread can do some work before calling `ConfigureThreadAndWaitForRun`. Once `ConfigureThreadAndWaitForRun` returns, the thread will be running with realtime restrictions if configured to do so. That means, any non-realtime work should be done before the `ConfigureThreadAndWaitForRun` call.

```cpp
aos::EventLoop *event_loop = ...;
std::thread worker1([event_loop]() {
    std::unique_ptr<aos::ThreadHandle> thread_handle = event_loop->ConfigureThreadAndWaitForRun("worker1");
    // This thread is realtime as long as thread_handle is alive.

});
std::thread worker2([event_loop]() {
    std::unique_ptr<aos::ThreadHandle> thread_handle = event_loop->ConfigureThreadAndWaitForRun("worker2");
    // This thread is realtime as long as thread_handle is alive.
});

// Run the event loop.
...

worker1.join();
worker2.join();
```

#### Ignoring some threads and creating others

It is allowed to ignore some threads and create others. For example, if you want to use only one thread from the configuration, you can ignore the other threads.

## Behaviour relative to the Event Loop

There is a two-way synchronization between the main thread running the event loop and the threads calling `ConfigureThreadAndWaitForRun`. The main thread will block until all non-ignored threads have called `ConfigureThreadAndWaitForRun`. The threads will block until the event loop has started running. Once the event loop starts running and the threads have called `ConfigureThreadAndWaitForRun`, the main thread and the additional threads will unblock and continue executing as normal.

```cpp
aos::ShmEventLoop *event_loop = ...;

// Ignore one thread, but create and configure the other.
event_loop->IgnoreThread("worker1");
std::thread worker2([event_loop]() {
    // This ConfigureThreadAndWaitForRun call will block until Run() is called on the event loop.
    std::unique_ptr<aos::ThreadHandle> thread_handle = event_loop->ConfigureThreadAndWaitForRun("worker2");

    // Now the thread is realtime and the OnRun() callbacks will be called in the
    // main thread.
});

event_loop->OnRun([]() {
    // This will be called in the main thread after all non-ignored threads have
    // been configured.
    ABSL_LOG(INFO) << "Threads have been configured.";
})

// Run the event loop. This will block until all threads have called ConfigureThreadAndWaitForRun.
event_loop->Run();

worker2.join();
```

The example above is also applicable to `aos::SimulatedEventLoopFactory` and its `Run()`, `RunUntil()`, and `RunFor()` methods.

## Special Considerations

### `ShmEventLoop`

When using threads with an `ShmEventLoop`, you must call `LockToThread()` from the main thread before you create any threads. This is necessary for some internal error checking.

```cpp
aos::ShmEventLoop *event_loop = ...;
event_loop->LockToThread();

// Create threads here.
std::thread thread(...);
```
