# ADR 0005: A thread-only event loop — simulation and unit tests without shared memory that outlives them

## Status

Accepted. Implemented in AOS: `QueueEventLoop` (`aos/events/queue_event_loop.h`) holds what was `ShmEventLoop`'s body, `ShmEventLoop` and `ThreadEventLoop` (`aos/events/thread_event_loop.h`) are the two types over it, and `ProcessLocalQueue` (`aos/ipc_lib/process_local_queue.h`) is the process-owned queue memory with its platform halves in `process_local_queue_{linux,darwin,windows}.cc`. Every `shm_event_loop_test` suite runs against both types. The mrccomm and allwpilib side -- picking `ThreadEventLoop` in `MRC_SimSystemServer_Initialize()` and in tests -- is not done yet. Written against `wpilib-release-2027` (2a16f9332), mrccomm `aos-control-data` (aa606b3) and allwpilib `aos_bridge` (3d11cfcb0).

## Context

`ShmEventLoop`'s channels are lockless queues in memory that every process on a `--shm_base` maps. That memory deliberately outlives every process that used it. `aos/ipc_lib/shm_mapping.h` makes it part of the contract: a queue nobody is using keeps its state, and the next process to attach finds it. On a robot that is what you want — a restarted process fetches the last message a peer sent before it came up, and starterd holds every channel open for its own lifetime anyway (`Starter::AddChannel`, `aos/starter/starterd_lib.cc`).

How long "outlives" is depends on the platform, and on two of them nothing ever ends it:

| Platform | What backs a queue | Lifetime |
| --- | --- | --- |
| Linux | A file on tmpfs under `/dev/shm/aos` (`shm_mapping_linux.cc`) | Until unlinked, or reboot. Visible with `ls`. |
| macOS | A path under `/dev/shm/` becomes a POSIX shm object named `/aos_<std::hash of the path>` (`shm_mapping_darwin.cc:28,38`); any other path is a file | Until `shm_unlink`, or reboot. No tool lists POSIX shm objects, so nobody can see what is left, and `--purge_shm_base` (`aos/starter/starterd.cc:27`) removes a directory these objects are not in. |
| Windows | A real file, `CreateFileA` with `FILE_ATTRIBUTE_TEMPORARY` (`shm_mapping_windows.cc:52`). The default `/dev/shm/aos` resolves to `\dev\shm\aos` on the current drive. | Until deleted. Survives reboot. `FILE_ATTRIBUTE_TEMPORARY` keeps the contents in cache while there is cache to hold them; the file is still on disk. |

Two users of AOS want none of this.

**WPILib desktop simulation.** Everything in sim that talks over AOS runs inside the robot program — possibly inside a JVM — through the one copy of AOS statically linked into libMrcLib:

- `halsim_ds_socket` calls `MRC_SimSystemServer_Initialize()` (allwpilib `simulation/halsim_ds_socket/src/main/native/cpp/main.cpp:208`), which starts `mrc::RobotDaemon` in-process (mrccomm `shipped/mrclib/src/SimSystemServer.cpp`). The daemon builds a `ShmEventLoop` and a `ControlSender`, with a comment saying simulation runs it inside the robot program (`mrccore/src/robotdaemon.cpp:1908-1913`).
- `MrcLibDs` calls `MRC_InitializeAllSubscribers` (allwpilib `hal/src/main/native/cpp/mrclib/MrcLibDs.cpp:320`), whose `aoc::ControlListener` runs its own `ShmEventLoop` on a thread (mrccomm `aoscore/src/ControlListener.cpp:20-28`).
- The Driver Station is another process, but it talks UDP/TCP to the in-process daemon, not AOS. Nothing starts `aosnt-bridge` or `aos_dump` against a sim.

Nothing in allwpilib sets `--shm_base`, so every sim run uses `/dev/shm/aos`. On macOS it leaves POSIX shm objects that cannot be listed; on Windows it leaves files that survive reboot. Two sims on one machine share queues, so each one's `RobotDaemon` sends control data the other one's listener receives.

**Unit tests.** mrccomm's `TestControlListener` and `TestControlDataTimeout` run a `ShmEventLoop` listener on a thread and send from the test thread — threads, never processes. To keep off `/dev/shm/aos`, a Catch listener points `--shm_base` at a per-PID temporary directory and removes it when the run ends (`aoscore/test/PrivateShmBase.cpp`). A crashed or killed run skips that and leaves the directory. Tests of libMrcLib.so have to reach its private copy of AOS through `MRC_SetAosShmBase`, because that library exports only `exports.txt` and its absl flags stay private to it.

AOS's own tests are already covered — `SetTestShmBase()` puts queues under `TEST_TMPDIR`, which Bazel cleans — and some of them (`ipc_stress_test.cc`) need real cross-process queues.

In both cases the processes never share queues with anyone. They pay for shared memory's lifetime and get none of its benefit.

## Decision

Add a second event loop type, `ThreadEventLoop` (name open), whose channels are ordinary memory owned by the process, shared between threads and never between processes. `ShmEventLoop` keeps its name, its meaning and its code path: `Shm` says shared memory, which says processes, and a loop that never leaves the process has no business being that type in disguise.

- **The OS reclaims everything when the process ends, however it ends.** There are no names, no files and no kernel objects to clean up, so there is no cleanup code and no per-platform lifetime protocol to get wrong.
- **Nothing outside the process can see the queues.** That is the cost, and for sim and tests it is also the point: two sims no longer share queues.
- **The two types share one implementation.** Everything above the queue memory — `LocklessQueue`, watchers, timers, phased loops, the `Aio` integration, realtime setup — is identical, so the implementation moves to a common base and each type supplies its queue memory.
- **A call site says which one it means.** Simulation constructs a `ThreadEventLoop`; the robot constructs a `ShmEventLoop`; both can exist in one process on purpose.

Not decided here: a shared-memory mode that frees queues once nobody maps them. It is weighed under "Alternatives considered" and kept for the case this ADR does not address — shared memory that a robot's processes can still see, but that does not outlive them.

## Design

### The seam: `MemoryMappedQueue`

The event loop reaches its queue memory in exactly one place: the `std::unique_ptr<ipc_lib::QueueMemory>` member of `SimpleShmFetcher` and `ShmSender` in `aos/events/queue_event_loop.cc`; watchers go through `SimpleShmFetcher`. Everything above it — `LocklessQueue`, readers, pinners, watchers, the wakeup path — takes a `LocklessQueueMemory *` plus a `LocklessQueueConfiguration` and does not care where the memory came from.

That member becomes a handle from the loop's queue provider:

- **`ShmEventLoop`**: today's `MemoryMappedQueue`, unchanged.
- **`ThreadEventLoop`**: one registry per process, keyed by channel name and type. The first lookup allocates the block, and later ones return it if their `LocklessQueueConfiguration` is equal to the one it was made with, which is stricter than the size check `MapShm()` makes. Every `ProcessLocalQueue` runs `InitializeLocklessQueueMemory()`, which only does anything the first time. The registry is mutex-protected on construction, destruction and `ResetAll()`, none of which is on a realtime path: construction already opens files today. A `fork()` waits for that mutex (`pthread_atfork()`), so a child forked while another thread makes or destroys a queue does not inherit it held. One per process is the starting point; if a test binary turns out to need several sets of queues, a namespace on the constructor is the revision.

### Two types over one implementation

`ShmEventLoop` is constructed by name in 117 files here, plus mrccomm and allwpilib, so it keeps its name and its API. The split is behind it:

- **`QueueEventLoop`** holds what is today `ShmEventLoop`'s body, and takes its queue provider from the derived class. The `queue_event_loop_internal` classes it friends keep their names until someone wants to rename them; nothing about them is shm-specific except the provider they hold.
- **`ShmEventLoop`** and **`ThreadEventLoop`** are thin derived types that supply a provider. Existing code keeps compiling unchanged, and code that accepts either takes a `QueueEventLoop &`.

The alternative shape, `ThreadEventLoop : public ShmEventLoop`, is a smaller diff and keeps every existing `ShmEventLoop *` working with no edit at all — mrccomm's `RobotDaemon` holds one. It is rejected anyway: it makes a loop with no shared memory an `is-a` of the type named for shared memory, which is the thing this feedback asked us not to do.

`--shm_base` and `--permissions` apply to `ShmEventLoop` only; `ThreadEventLoop` reads neither.

### Memory: page-aligned, zeroed, pre-faulted, mapped twice

`InitializeLocklessQueueMemory()` expects zeroed memory, and the event loop expects the pages pre-faulted, so the allocation comes from the page allocator, one mapping per platform as below, and still goes through `PageFaultDataWrite()`.

Today each queue is mapped twice, read/write for senders and read-only for readers, so a reader that writes into the queue faults instead of corrupting it. That is worth keeping in the type meant for development. Each platform can map anonymous memory twice without giving it a name:

- **Linux:** `memfd_create`, mapped twice.
- **macOS:** `mmap(MAP_ANON | MAP_SHARED)`, then `mach_vm_remap` of that region with `copy=FALSE`, then `mach_vm_protect` read-only.
- **Windows:** an _unnamed_ page-file-backed section (`CreateFileMapping(INVALID_HANDLE_VALUE, …, NULL)`), mapped twice.

All three disappear with the process. The read-only view is there from the first change, not added later: a reader writing into a queue is exactly the bug this type's users, sim and tests, exist to catch.

### Lifetime inside the process: until exit

A registry entry lives until the process exits, not until its last user is destroyed. That keeps the persistence contract inside the process: a listener thread that restarts, or a test that destroys one event loop and builds another, still fetches the last message. `ProcessLocalQueueTest.MessageOutlivesUsers` checks exactly that. Memory is bounded by the number of distinct channels, which is what shared memory costs today. `ProcessLocalQueue::ResetAll()` empties every queue when none has a live user; the thread-kind `shm_event_loop_test` factories call it to start each test from a clean slate, as the shm ones delete their queues.

### Choosing a type, in code

There is no mode to set and nothing to latch. A program picks a type where it builds the loop, and two programs that pick differently cannot silently half-share anything: they simply have different channels.

Simulation has two construction sites in mrccomm: `RobotDaemon`'s `AosLoop`, a `std::unique_ptr<aos::ShmEventLoop>` member (`mrccore/src/robotdaemon.cpp:226,1911`), and `ControlListener`'s stack loop (`aoscore/src/ControlListener.cpp:28`). Both run on the robot too, so each needs the type chosen at runtime — a `std::unique_ptr<aos::QueueEventLoop>` from a small factory in aoscore. The member's type changes; `ControlListener` holds a pointer instead of a stack loop.

Sim already has the one place to make that choice. `MRC_SimSystemServer_Initialize()` (`shipped/mrclib/src/SimSystemServer.cpp:81`) exists only for sim, is what `halsim_ds_socket` calls first, and constructs the `RobotDaemon` itself — `MrcSimSystemServer`'s constructor calls `InitAndStartComms()`, which is where `AosLoop` is built. Selecting the thread type at the top of that constructor, before `new RobotDaemon`, covers the daemon's loop, and `ControlListener` is built later by `MRC_InitializeAllSubscribers()`. No new call from allwpilib is needed.

That is the real cost of a type over a flag: a flag would have left both construction lines alone. It buys a call site that says which world it is in, and it drops the rule that the flag must be set before the first loop is built.

Tests replace `PrivateShmBase` with the type. Tests that go through libMrcLib.so need it selected behind an exported entry point, the way `MRC_SetAosShmBase` already is.

### Behind libMrcLib's symbol hiding

libMrcLib exports only the C entry points in `shipped/mrclib/exports.txt`: a version script with `local: *` on Linux, a `.def` on Windows, an exported-symbols list on macOS (`cmake/GenerateExports.cmake`). Every AOS symbol, flags included, is private to the library, which is why `MRC_SetAosShmBase` exists at all.

This design fits that, and mostly because the choice is made inside the library:

- **Sim needs no new export.** `RobotDaemon` and `ControlListener` are compiled into libMrcLib, and `MRC_SimSystemServer_Initialize()` is already exported. The factory that picks `ThreadEventLoop` lives next to them in aoscore, keyed on that call having run. Nothing outside the library names an AOS type.
- **Tests through libMrcLib.so need one new export**, a sibling of `MRC_SetAosShmBase` that selects the type, because `MRC_SimSystemServer_Initialize()` also starts the daemon and an NT server, which is more than `MrcLibTestShmBase` wants. Tests that link `MrcLibStaticBase` or `AosCore` directly call the C++ API.
- **The registry is a hidden static.** It lives in the library's copy of AOS and nothing outside can reach it, which is what "process-local" should mean.
- **Hiding is what makes two copies of AOS genuinely separate.** With `local: *` there is no interposition: a host that linked its own AOS would get its own registry, and its `ThreadEventLoop` could not see libMrcLib's channels. Today the host has none, so nothing changes. If one ever appears, the only way across is another exported C entry point, or shared memory — the same choice the persistent path already faces.
- **Nothing here is platform-specific.** The `.def` and the macOS list hide the same symbols the version script does.

### What does not change

- **Wakeups** already work inside a process. `ThreadSignalSender` targets `(pid, tid)` (`aos/ipc_lib/lockless_queue.cc:803,881`): `rt_tgsigqueueinfo` on Linux, the `Local\aos-wakeup-<pid>-<tid>` event on Windows (`thread_signal_windows.cc:18`), and `kill(pid, SIGUSR1)` on macOS (`thread_signal_darwin.cc:31`). A loop that never leaves the process could later wake threads without signals — attractive in a JVM — but that is a separate change.
- **Ownership tracking** already covers a thread dying without its process; process death stops mattering.
- **The UID check** in `InitializeLocklessQueueMemory()` (`lockless_queue.cc:701`) is trivially satisfied.
- **`fork()`**: both views are shared mappings (a memfd on Linux, a `VM_INHERIT_SHARE` remap on macOS), so a forked child shares the parent's queues exactly as it would shared memory. gtest death tests fork, and they are fine for the same reason they are on shared memory.

## Consequences

- **Sim and tests leak nothing on any platform, including after a crash.** No per-OS cleanup code, and nothing for a user to find in `/dev/shm`, `%TEMP%` or the invisible POSIX shm namespace.
- **Two sims on one machine are isolated.**
- **`ShmEventLoop` is untouched, and so is what its name means.** Normal users pay nothing and see no behavior change.
- **`ShmEventLoop`'s body moves to a base class, in the same change as the new type.** That is a mechanical change to a class 117 files here construct, plus mrccomm and allwpilib, and it is the one risk in this ADR worth reviewing carefully. Nothing in the public API moves. It lands together with `ThreadEventLoop` rather than as a preparatory refactor, so the split is reviewed against the thing that needs it.
- **The two types can be used together deliberately.** A future in-process bridge could hold a `ThreadEventLoop` for sim's channels and a `ShmEventLoop` for channels outside tools can see, and copy between them. The mode-flag design could not express that at all.
- **Nothing outside the process can attach to a sim.** `aos_dump` or `aosnt-bridge` pointed at a sim would find, or create, empty shared queues and see nothing, with no error. Nothing does that today, and the bridge above is the answer if it ever has to.
- **Two copies of AOS in one process do not see each other's queues.** Today they would, through the same `/dev/shm` files. No current configuration loads two — sim's only copy is libMrcLib's; `aosnt` links only into `aosnt-bridge` — but a future host that links AOS itself as well as libMrcLib would split silently.
- **Sim's construction sites choose a type at runtime**, so they hold a base-class pointer rather than a concrete loop.

## Alternatives considered

- **A shared-memory mode that frees queues once nobody maps them.** Same opt-in, but the memory stays in shared memory:
  - **Windows:** named page-file-backed sections, which the kernel frees when the last handle closes. The section handle has to stay open for the mapping's life, because a named object's name disappears with its last handle even while views remain: after `CloseHandle()` with a view still mapped, `OpenFileMappingW()` fails with `ERROR_FILE_NOT_FOUND`. Names cannot contain `\` and have to pick `Local\`, as the wakeup events already do.
  - **macOS:** POSIX shm has no reference count, and SysV shm's default limits (on the order of 4 MB total) rule it out, so the kernel cannot do it. Instead, a `flock` on `/tmp/aos-<hash>.lock` — shared while attached, a try-exclusive on the way out — plus a list of object names to `shm_unlink`. A crash of the last process leaves memory until the next first attach purges it, or a forked reaper is added.

  It keeps outside tools working. It costs two protocols that differ by platform and must fail loudly when a persistent-mode process and a released-mode process share a base, and cleanup is still not guaranteed on macOS. Worth building only if sim has to be observable from another process.

- **A mode on `ShmEventLoop`, set by a flag.** This ADR's first draft. The same provider swap, chosen by process-wide state latched at the first queue, so no call site changes. Rejected for what the name would then mean: a `ShmEventLoop` that has no shared memory, and a `--shm_base` and `--permissions` that silently do nothing. The mechanism has its own problems too — the setting must be latched, it has to be set before the first loop is built, a copy of AOS that never sees the setter disagrees silently, and a process can never run both kinds at once, which rules out the in-process bridge above. It is a smaller diff, and that is all it has.
- **A thread-only event loop that reimplements the queue** with a mutex and a condition variable instead of `LocklessQueue`. Rejected: sim and tests should exercise the queue code the robot runs, and everything above the memory — watchers, pinning, fetchers, timing reports — already works unchanged.
- **`SimulatedEventLoopFactory`.** Deterministic and single-threaded, and already what mrccomm's `TestControlChannel` and aosnt's `NtBridgeTest` use. It fits tests of logic; it does not fit sim, which runs in real time on several threads, or tests whose point is that listener thread.
- **Per-run `--shm_base` plus cleanup on exit.** What `PrivateShmBase` does today. It leaks on every crash, and on macOS a `/dev/shm/` base would put queues where no cleanup can list them.
- **Shared-memory cleanup by starterd.** Sim has no starterd.

## Verification

- **Contract suites:** instantiate `shm_event_loop_test.cc`'s `AbstractEventLoopTest` and `ShmEventLoopTest` suites against `ThreadEventLoop`, on Linux, macOS and Windows. They already run several loops in one process, which is exactly this type's scope.
- **`ShmEventLoop` is unchanged:** its existing suites pass after the base-class extraction, which is how that refactor is held honest.
- **Nothing left behind:** after a test process exits, both normally and by `abort()`, no `/dev/shm/aos` entry, `shm_base` file or `/aos_*` POSIX shm object exists.
- **The two types do not see each other:** a `ThreadEventLoop` and a `ShmEventLoop` on the same channel, in one process, do not exchange messages.
- **mrccomm:** `TestControlListener` and `TestControlDataTimeout` pass on `ThreadEventLoop` with `PrivateShmBase` removed. A sim run alongside a second concurrent sim shows no control data crossing between them.

## Open questions

- **The name.** `ThreadEventLoop`, `LocalEventLoop`, `InProcessEventLoop`. And what the base is called, if not `QueueEventLoop`.
- **Signal-free wakeups** for a loop that never leaves the process, as a follow-up.

## References

- `aos/ipc_lib/shm_mapping.h` — the lifetime contract
- `aos/ipc_lib/shm_mapping_linux.cc`, `aos/ipc_lib/shm_mapping_darwin.cc`, `aos/ipc_lib/shm_mapping_windows.cc` — the three backends
- `aos/ipc_lib/queue_memory.h` — the seam
- `aos/ipc_lib/memory_mapped_queue.h`, `aos/ipc_lib/process_local_queue.h` — the two kinds of queue memory
- `aos/events/queue_event_loop.cc` — `SimpleShmFetcher`, `ShmSender`
- `aos/ipc_lib/lockless_queue.cc` — `InitializeLocklessQueueMemory()`, watcher wakeups
- `aos/ipc_lib/thread_signal.h` — cross-thread wakeups
- `aos/starter/starterd.cc`, `aos/starter/starterd_lib.cc` — `--purge_shm_base`, starterd holding every channel
- mrccomm `mrccore/src/robotdaemon.cpp`, `shipped/mrclib/src/SimSystemServer.cpp`, `aoscore/src/ControlListener.cpp`, `aoscore/test/PrivateShmBase.cpp`
- allwpilib `simulation/halsim_ds_socket/src/main/native/cpp/main.cpp`, `hal/src/main/native/cpp/mrclib/MrcLibDs.cpp`
