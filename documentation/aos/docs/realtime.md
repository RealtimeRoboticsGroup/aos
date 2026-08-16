# How AOS is Real-Time

This document explains the mechanisms that make an AOS system real-time, and _why_ each of them exists. Most of the design decisions below are the answer to a specific way a deadline can be missed.

For how to confirm any of this on real hardware, see [Measuring Real-Time Performance](measuring_realtime.md).

## What "real-time" means here

AOS targets **hard real-time on Linux with `PREEMPT_RT`**. Concretely, the promise is:

> Once an application is running, every operation it performs on the critical path has a bound on how long it can take that does not depend on what any other process on the system is doing.

That is a different claim from "it's fast". A queue that averages 2 µs but occasionally takes 40 ms — because a low-priority process got descheduled while holding something the reader needs — is fast on average and not real-time. What matters is the tail, so that is what the design optimizes for, sometimes at the cost of the average.

Two things follow from that definition, and they shape everything else:

1. **The bound only applies once you are running.** Setup is explicitly _not_ real-time. AOS draws a line between "starting up" (allocate, mmap, attach to queues, clean up after dead processes, take locks) and "running" (no allocation, no blocking, no unbounded loops), and pushes everything expensive to the non-RT side of it.
2. **Anything that can block on a lower-priority task has to be eliminated, not just made fast.** Priority inversion doesn't show up in average-case measurements, so making it impossible by construction is more useful than measuring for it.

## What can cost you a deadline

| Failure | Where it comes from |
| --- | --- |
| Priority inversion | A high-priority task waits on something a low-priority task holds |
| Page faults | Memory that isn't resident when the RT thread touches it |
| Allocation | `malloc` takes a lock in glibc, and can call into the kernel |
| Unbounded work | Loops over "everything", cleanup of dead processes, filesystem access |
| IRQ / softIRQ stealing CPU | Network, USB, MMC interrupt handlers running above your RT threads |
| Scheduler latency | Non-preemptible sections in the kernel — mostly the RT patchset's problem, not ours |
| Crashed processes leaving state behind | A killed sender that was midway through a queue write |

The rest of this document walks the layers of AOS and shows what each one does about these.

---

## Layer 1: Process and thread setup

`aos::ShmEventLoop::Run()` (`aos/events/shm_event_loop.cc`) is where an application crosses from non-RT setup into RT operation. In order, it:

1. Registers signal handlers and the wakeup signalfd receiver.
2. Schedules timing reports.
3. **Constructs every watcher** — this is the expensive part, see [Layer 4](#layer-4-startup-and-connection-costs).
4. Waits for all configured worker threads to check in.
5. Calls `InitializeRealtime()` — which is `aos::InitRT()` (`aos/realtime.cc`) — locking memory and setting the RT rlimits.
6. Calls `SetCurrentThreadRealtimePriority()`, which sets `SCHED_FIFO`/`SCHED_RR` **and** arms the malloc hooks.
7. Snaps every watcher's queue index to the newest message — deliberately done _after_ going RT, so the window in which a message can be missed is as small as possible.
8. Runs the `OnRun` handlers, then enters the event loop.

On exit it drops back to `SCHED_OTHER` before doing any teardown.

### Memory locking

`InitRT()` → `LockAllMemory()` does five things:

- Raises `RLIMIT_MEMLOCK` to infinity, then `mlockall(MCL_CURRENT | MCL_FUTURE)`. Everything mapped now, and everything mapped later, stays resident.
- `mallopt(M_TRIM_THRESHOLD, -1)` — never give freed memory back to the OS.
- `mallopt(M_MMAP_MAX, 0)` — never satisfy a large `malloc` with a fresh `mmap`.
- Pins tcmalloc's release rate to zero, if tcmalloc is linked in.
- Touches 32 KB of stack and allocates, touches, and frees a 512 KB heap chunk, so those pages are resident and the allocator has a warm pool.

The failure this prevents is subtle: without it, an allocation the kernel satisfies lazily produces a _page fault at first touch_ — which happens inside your control loop, not at allocation time. Servicing the fault may need `mmap_lock`, which a non-RT thread might hold. That is a page fault turning into an unbounded priority inversion.

`InitRT()` then sets two rlimits:

- **`RLIMIT_RTTIME` = 3 s.** An RT thread that runs for 3 seconds of solid CPU without blocking gets `SIGXCPU`. This is a deliberate safety net: a runaway `SCHED_FIFO` thread at high priority on a single-core system is otherwise unrecoverable without a power cycle.
- **`RLIMIT_RTPRIO` = 40.** Unprivileged processes cannot go above priority 40 by default. This is the ceiling AOS reserves the space above for kernel IRQ threads and anything privileged.

`--skip_locking_memory` bypasses the locking, and `--skip_realtime_scheduler` skips both the rlimits and the actual scheduler change. Both exist so tests and desktop debugging work without privileges. Neither belongs on a deployed target.

### The malloc hooks

`aos/realtime_linux.cc` interposes `malloc`, `free`, `realloc`, and `calloc` (as weak aliases, plus tcmalloc's `MallocHook_*` API when tcmalloc is present). When the current thread is marked realtime and `--die_on_malloc` is set (the default), any of them **crashes the process immediately** with a backtrace.

This catches a lot in practice. Allocation in glibc takes an arena lock that non-RT threads in the same process also take, and can fall through to `brk`/`mmap`. It is easy to introduce with an ordinary `std::string` or `std::vector::push_back` in a message handler, and hard to find by measurement, since it costs 100 ns nearly every time and only occasionally more. Turning it into a crash moves the discovery from a rare latency spike to the first run.

The RT/non-RT state is a thread-local flag, not a query of the actual scheduler policy, which means it works identically in simulation and in tests. That in turn means the RT constraints of an application are exercised by its unit tests on a desktop, not just on the target.

Related API in `aos/realtime.h`:

- `aos::CheckRealtime()` / `aos::CheckNotRealtime()` — assert which side of the line you are on. Used to mark APIs that are _not_ allowed on an RT path (`Aio` destruction, for instance, because it can free).
- `aos::ScopedNotRealtime` — an explicit, auditable escape hatch, and the one you will actually reach for. Logging a rare error from an RT thread is a legitimate use; doing it every cycle is not.
- `aos::ScopedRealtime` — the inverse, applying the restrictions to a scope. Public, but rarely needed: people generally want to carve out an exception rather than add enforcement.
- `RealtimePolicy::NO_MODE` — take the RT scheduling priority without entering realtime mode at all, so the malloc hooks never fire. This makes nothing safe; it only stops AOS from catching it. It exists for when you are willing to accept the risk — a library you can't fix, say — and the latency that follows is yours to own.

### Dying drops to `SCHED_OTHER` first

A crashing RT process is a latency hazard in its own right. Between deciding to die and actually dying, it formats a message, symbolizes a backtrace, writes to stderr, and possibly dumps core — none of that bounded, and all of it at RT priority on a core that healthy control loops may be sharing. A process that is already broken should not get to starve the ones that aren't.

So AOS leaves the RT scheduler _before_ doing any of that work. `aos_FatalUnsetRealtimePriority()` (`aos/realtime.cc`):

- Drops the current thread to `SCHED_OTHER` at priority 0 and clears the RT-mode flag, first, before anything else.
- Walks `/proc/self/task` and puts **every** thread in the process on `SCHED_OTHER` — not just the one that is dying. Sibling worker threads are still at RT priority and are about to be killed anyway; leaving them there while the process unwinds serves nobody.
- Saves and restores `errno` around all of it, since it runs inside failure paths that are about to report it.

It is wired in through weak symbols patched into abseil (`third_party/abseil/0001-Add-hooks-for-using-abseil-with-AOS.patch` and `0003-Drop-out-of-RT-before-dying-from-signals-and-raw-log.patch`), from four places:

- `CheckOpMessageBuilder`'s constructor, covering `ABSL_CHECK_EQ` and friends.
- `LogMessage`'s constructor, when the severity is `kFatal`, covering `ABSL_LOG(FATAL)`, `ABSL_CHECK`, and `ABSL_PCHECK`.
- `RawLogVA`, when the severity is fatal, covering `ABSL_RAW_LOG(FATAL)` — which is what the malloc hooks use.
- The top of abseil's failure signal handler, so `SIGSEGV` and friends drop before any symbolization starts.

In every case the hook runs before the message is built or the backtrace symbolized, which is the point. And every hook is weak, so a non-AOS binary linking the same abseil gets a no-op.

### Priorities and affinity belong in config, not code

Priority, scheduling policy, and CPU affinity come from the `Application` entry in the AOS config (`aos/configuration.fbs`):

```json
{
  "name": "drivetrain",
  "scheduling_policy": "SCHEDULER_FIFO",
  "priority": 33,
  "cpu_affinity": [2]
}
```

`EventLoop::ParseSchedulingSettings()` reads these at construction, and again whenever `set_name()` changes which application entry applies.

#### Why config

Priority 33 means nothing on its own — only "above the thing at 30, below the thing at 37". The right value depends on what else is on the machine, how many cores it has, and where the IRQ threads sit. That is a property of the target, not of the application, and the same binary gets deployed to targets that answer it differently.

So the number lives in the per-target config, and the binary stays identical everywhere. `Configuration.imports` merges the importing config over a base one, application-by-application and field-by-field (`UnpackApplication`, `aos/configuration.cc`), so overriding one priority is:

```json
{
  "imports": ["../common/aos_config.json"],
  "applications": [{"name": "drivetrain", "priority": 41}]
}
```

Lookup is per node (`GetApplication(configuration_, node_, name_)`), so one config can also give an application different priorities on different nodes. And it keeps config as the source of truth, which is what makes a `dump_rtprio` audit mean anything.

#### The in-code override

`SetRuntimeRealtimePriority()` and `SetRuntimeAffinity()` before `Run()` win, since they run after `ParseSchedulingSettings()`. Plenty of applications still hardcode a priority this way, simply because they predate the config fields and nobody has converted them yet. It remains the right call for a flag-driven tool (`aos_jitter --priority`), a library default for use outside a configured system, or a test; for anything running under `starter`, the number belongs in the config.

Worker threads are declared in the same `Application` entry and configured via `ConfigureThreadAndWaitForRun()`; see [Threading](threading.md). IRQ and kernel-thread priorities are configured the same way, in a sibling file ([Layer 6](#irqs-and-kernel-threads)).

---

## Layer 2: The IPC primitive

The core of AOS's real-time story is `aos/ipc_lib/lockless_queue.cc`. Each channel is one queue in its own shared memory mapping. The design goals:

- Lockless reads _and_ writes, so that a `SCHED_OTHER` process being descheduled mid-write cannot stall an RT reader.
- Any process dying at any point leaves the queue usable.
- O(1) random access, so a fetcher can jump to "latest" without walking.
- New senders and watchers can attach without disturbing RT traffic already flowing.
- Zero-copy: flatbuffers are built directly in the shared memory slot.

### Every message slot is the same size

A channel's `max_size` fixes the size of every message slot in its queue, and all of them are allocated at queue creation. A 12-byte message occupies the same slot a 2 KB one would.

That wastes memory, deliberately, to make fragmentation impossible. Variable-sized messages would mean a real allocator inside shared memory: free lists, coalescing, best-fit searches — work whose cost depends on the history of every send that came before it, and which can fail late with plenty of free space but no contiguous run. None of that is boundable, and all of it would sit on the send path. Uniform slots replace it with an array index.

It is also what makes the swap below work. Any message fits in any slot, so a sender can exchange the one it holds for the one in the queue without checking anything.

### There is no free list

The queue is an array of `Index`es pointing into a pre-allocated array of those fixed-size messages. Each sender permanently owns one scratch message. **Sending is a swap**: the sender fills out its scratch message, then atomically exchanges its index with the index sitting in the target queue slot, and now owns the message that used to be there.

The free list is absent not because one would be awkward to write, but because there is nothing left for it to track. A real-time system has to be sized for the worst case regardless: the queue depth, one scratch message per sender, one per pinner. Do that arithmetic up front and the total message count is fixed at queue creation — and every message is then permanently accounted for by exactly one owner. It is sitting in a queue slot, held as some sender's scratch, or pinned. There is no fourth state, so there is nothing to keep a list of.

That is what makes the swap total rather than conditional. A sender never asks for a message, because it already has one; it only ever trades. No allocation happens, there is no free list to corrupt, and there is no allocator lock to invert on.

### Detecting torn reads instead of preventing them

Readers do not lock. A message can be overwritten while it is being read, and AOS's contract is that this is _detectable_, not preventable:

- Every message carries the `QueueIndex` it was written as.
- A message in slot `X` is valid only if its `QueueIndex` matches `X` both before and after you read the body, with the appropriate barriers.
- If it doesn't match, the reader can distinguish "read past the end" (index is exactly `queue_size` less than expected) from "fell too far behind", and reports `TOO_OLD` / `OVERWROTE` to the caller.

The `Index` deliberately packs the message-array index into the low 16 bits and the low 16 bits of the queue index into the high 16 bits, so that a slot being reused by a wrapped-around queue is distinguishable from the same message reappearing.

Detection is bounded and lockless; what happens _afterwards_ is neither, and that is fine. A fetcher that reports `TOO_OLD` or `OVERWROTE` has been lapped by the queue, which means it has already missed however many messages the queue is deep. Its deadline is long gone. So AOS spends the time to publish a timing report and then `LOG(FATAL)`s with the channel name — unbounded work, on the assumption that there is no longer a real-time guarantee left to protect. The RT budget is for the paths that still have a deadline to make.

### Writing is a transaction that anyone can finish

Publishing touches several independent words of shared memory: the queue slot, `next_queue_index`, and the sender's own bookkeeping. There is no instruction that updates all of them together, and taking a lock to cover them is exactly what this design refuses to do. **So publishing cannot be atomic**, and no amount of care makes it so.

What the queue actually owes its users is weaker than atomicity, and achievable: **the state must never be observably inconsistent**. Nobody is promised that a publish happens all at once — only that at every instant, any reader sees a coherent queue, and any process arriving later can tell what was in flight and finish the job. A writer preempted mid-publish, or `SIGKILL`ed mid-publish, must not leave anything a subsequent reader or writer can be confused by.

That is what a transaction gives you. Each sender has a scratch slot in shared memory recording both the message it is filling out and the index it is about to swap with, written _before_ the swap. Any other process that comes along can therefore roll the transaction forward or backward in constant time, using only what is in shared memory and without the original writer's cooperation.

Concretely a send is:

1. Read `next_queue_index`.
2. Force the _previous_ message's send times to be populated, if they aren't already.
3. Check whether the slot `next_queue_index` points to is from the previous wrap or the current one; if the previous writer hadn't yet bumped `next_queue_index`, increment and retry.
4. Invalidate this message's send times.
5. Record the message and queue index being replaced into the sender's scratch space.
6. Compare-and-exchange the new index into the queue slot. **The message is now sent**; everything past here is cleanup.
7. Populate the send times.
8. Bump `next_queue_index`. If that fails, someone else already helped.
9. Clear the scratch space.

### The IPC layer owns the timestamp

**The queue stamps the message, not the application.** Steps 2, 4, and 7 above are how, and the ordering is the whole point: the stamp is taken _after_ the message is formally in the queue, not before.

That is backwards from the obvious implementation, and deliberately so. Stamping before publishing lets a timestamp predate the moment anyone could observe the message, which breaks cross-channel ordering. The failure is worth spelling out — A sends on queue 1, B sends on queue 2, C reads both:

1. A timestamps its message for queue 1.
2. B timestamps its message for queue 2.
3. B publishes on queue 2.
4. C wakes up and observes B's message on queue 2.
5. C checks queue 1 for anything that arrived earlier. A hasn't published, so it sees nothing.
6. A publishes on queue 1.
7. C observes A's message — carrying a timestamp _earlier_ than the one it already processed, and earlier than what it saw when it checked in step 5.

From C's point of view time ran backwards, and a log replayed by timestamp would reorder the two messages relative to what any observer actually saw. Taking the stamp after the publish makes that impossible: a timestamp can never precede visibility.

The catch is that a sender can be preempted, or killed, between publishing and stamping — so the message is visible with no timestamp on it. The queue resolves this by making the stamp **whoever-gets-there-first**, not sender-only. `Message::SetSendTimes()` compare-and-exchanges the clock reading into the header, so the first caller wins and everyone afterwards reads back the same value. Readers call it too, from `Read()` — a fetcher that finds an unstamped message stamps it. This is what makes "the time the message became visible" literal rather than approximate: if the sender didn't get there, the time recorded is the first moment an observer actually saw it.

Monotonicity within a channel then falls out of step 2. Before claiming its own slot, a sender first forces the previous message's times to be populated. Message N is therefore always stamped before N+1 can be, so stamps come out in queue order no matter how many senders are racing. `SendRace` in `lockless_queue_test` is the test that holds this.

One consequence: since the real send time isn't available until after the send, the "sent too fast" check can't use it. It uses a `monotonic_clock::now()` sampled beforehand as a conservative estimate, which is enough to guarantee readers never _observe_ messages sent too fast.

#### The 32-bit fallback

All of the above needs a lock-free 64-bit atomic, which 32-bit platforms don't have. There, `AOS_IPC_LIB_LOCKLESS_QUEUE_HAS_ATOMIC_TIME_POINT` is unset, the header holds plain time points instead of atomic ones, and the queue writes them the naive way, before the swap. The cross-channel race above is knowingly left unresolved — the price of the platform, not a design position.

Which mode you get is a build-time platform decision, not a runtime probe: `//aos/ipc_lib:lockless_queue` selects `_no_atomic_times` for the roboRIO (armv7) and `_atomic_times` for everything else. So the correct behavior is what you get by default, and the fallback is confined to one increasingly rare 32-bit target. Both are built and tested (`lockless_queue_test_atomic_times` and `lockless_queue_test_no_atomic_times`), so the fallback keeps working for as long as anything needs it.

### Death detection takes two mechanisms

Every sender, watcher, and pinner slot is owned via a `RobustOwnershipTracker` (`aos/ipc_lib/robust_ownership_tracker.h`). The obvious mechanism is a Linux **robust futex**: the kernel sets an owner-died bit in the futex word when the owning thread exits, so a slot whose owner was `SIGKILL`ed is identifiable with a single relaxed load, without any cooperation from the dead process.

That works most of the time, and it is not sufficient on its own. Two ways it fails:

- **The bit may never get set.** The kernel's robust-list cleanup at thread exit is best-effort. It depends on the robust list being intact and the futex memory still mapped at that moment; if the mapping is gone or the list is damaged, the walk faults and the kernel silently gives up. Nothing reports that this happened.
- **The futex word only holds a TID, and TIDs get recycled.** A stale TID that now names a live, unrelated thread reads as perfectly alive.

Either one leaks a slot permanently, so the tracker carries a second, independent piece of evidence: the owner's **start time in ticks**, read from `/proc/<tid>/stat` and stored next to the futex when the slot is claimed. `OwnerIsDefinitelyAbsolutelyDead()` (`robust_ownership_tracker_linux.cc`) uses the futex bit as a fast path, and when it isn't set, goes and looks:

- No `/proc` entry for that TID at all → the owner is gone.
- An entry exists, but its start time doesn't match what was recorded → the TID was recycled, so the original owner is gone.

`NoMatchingPID` and `NoMatchingStartTime` in `robust_ownership_tracker_test` pin both cases, each asserting that the futex still says "alive" while the definitive check correctly says "dead".

`Acquire()` has an ordering requirement that follows from this: the start time must be written and visible **before** the futex is claimed. Otherwise a concurrent observer can see a claimed futex next to unset metadata and wrongly conclude the owner is dead. Concurrent `Acquire()` calls would also race each other's metadata, so they have to be serialized — which is what the queue setup lock does.

#### Which check runs where

The two checks are not interchangeable, and the split is what keeps this affordable:

- **The RT wakeup path uses only the futex bit**. It is a relaxed load of shared memory. Reading `/proc` is filesystem work and has no place on a send. Being occasionally wrong here is harmless: the cost of believing a dead watcher is alive is one wasted `rt_tgsigqueueinfo`, and `Signal()` already tolerates `ESRCH` for exactly this reason.
- **Every recovery and cleanup path uses the definitive check** — sender cleanup, pinner cleanup, and claiming a watcher slot. All of those run at attach time, off the RT path, under the setup lock, where reading `/proc` for a handful of slots costs nothing that matters.

This is the same trade as everywhere else in the queue: pay for certainty in the non-real-time path that has time for it, and let the real-time path run on the cheap approximation whose failure mode is bounded.

Note the constraint the futex imposes: PI futexes are kernel-enforced to be unlocked by the same thread that locked them, so **senders, watchers, and pinners must be destroyed on the thread that constructed them**. This is discussed further in `documentation/adr/0001-aio-io-uring-single-issuer.md`.

### Redzones

Each message is bracketed by redzones filled with a position-derived pattern. A sender that writes past its buffer is caught on `Send()` with `INVALID_REDZONE`, which is reported through timing reports as a send error rather than silently corrupting a neighbor's message.

---

## Layer 3: Wakeups and priority inversion

Getting a message into shared memory is the easy half. **Waking the subscribers up is where the real-time behavior is actually won or lost**, and it is the part most IPC systems get wrong.

### Why signals

The wakeup mechanism was picked by measuring four candidates on a roboRIO, sender and receiver pinned to the same core with the receiver at lower priority:

| Mechanism         | Max latency (µs) | Average latency (µs) | Has an fd |
| ----------------- | ---------------- | -------------------- | --------- |
| Futex             | 334.0            | 137.6                | no        |
| Named pipe        | 311.6            | 101.4                | yes       |
| Signal + signalfd | 334.2            | 82.7                 | yes       |
| eventfd           | 273.9            | 82.6                 | yes       |

The maxima are all the same, and the averages are all close enough not to matter. The decision was made on the _other_ properties:

- **Futexes have no file descriptor**, so waiting on one requires a dedicated thread. That makes every AOS application multithreaded on day one, which contradicts the single-threaded event loop model.
- **Named pipes are point-to-point.** A new subscriber means the sender must discover it, find its pipe, and `open()` it — from the RT send path.
- **eventfds are worse still**: the fd itself has to be distributed, through a broker process, `starter`, or `/proc` scraping. Again, on the RT path.
- **Signals need only a PID/TID pair**, which is trivially placed in the shared memory watcher table, and they have a file descriptor on the receiving side via `signalfd`. Nothing needs to be discovered or opened when a new subscriber appears.

AOS uses `SIGRTMIN + 2` (`kWakeupSignal`, `aos/ipc_lib/thread_signal.h`), delivered thread-directed with `rt_tgsigqueueinfo(2)`, received on a `signalfd` that the event loop watches like any other fd.

### The wakeup carries no information

A signal cannot tell you _which_ queue has a message. That looks like a limitation and is closer to the opposite: the receiver, on waking, simply checks every one of its watchers. An application subscribes to a handful of channels, so a full scan is cheap — and giving up on identifying the channel buys back several much harder problems.

**Ordering stops depending on the wakeup.** If a wakeup meant "channel X has a message", then preserving order would require that wakeup to be delivered atomically with the publish, and ordered against every other publish on the machine. A receiver told about X could otherwise act on it before being told about an earlier message on Y. There is no way to do that across processes without a global lock on the send path.

Because the wakeup says nothing, none of that is needed — and what you get back is stronger than just dropping a requirement. **Delivery order no longer depends on wakeup timing at all.** A wakeup can be late, or lost entirely, or be a wakeup for some completely different channel, and the application still receives every message in the order it was published.

The mechanism is that the message is in the queue before the signal is sent, so a scan finds the true current state of every queue regardless of what prompted it; the events are then handled out of a heap ordered by send timestamp (`event_loop.cc`), and those timestamps are assigned after publish and are monotonic (see [The IPC layer owns the timestamp](#the-ipc-layer-owns-the-timestamp)). The effect is what matters: a delayed wakeup makes everything happen later, but it cannot reorder anything, and a message found by an unrelated scan is still delivered in its correct place relative to everything else. The wakeup is a hint that there is work to do, and carries no other meaning.

**Missed wakeups are largely self-healing.** A wakeup that gets lost or coalesced away doesn't strand its message: the next wakeup from _any_ channel triggers a full scan that picks it up, as does any timer firing or fetch. This is also why duplicate signals are harmless — a redundant scan finds nothing new — which is what makes coalescing them safe to want.

**The cost is that it hides delivery failures.** The same property that recovers a lost wakeup also means a genuinely dropped signal produces no distinctive symptom. It surfaces as latency on one message, until something unrelated wakes the process, and it is essentially undiagnosable after the fact. That is the reason a failed signal send is fatal rather than absorbed (see [The PID/TID reuse race](#the-pidtid-reuse-race)): the error at the send is the only opportunity to notice.

### Low-priority sender → high-priority subscriber

This is the case the user of an IPC system worries about first, and it is handled by the queue being lockless. A `SCHED_OTHER` sender that gets descheduled in the middle of a write does not block an RT reader: the reader either sees the old message or the new one, detects a torn read if it raced, and moves on. There is no lock for the low-priority sender to be holding.

The wakeup itself is a `rt_tgsigqueueinfo` syscall per watcher, done in the sender's context. A low-priority sender being slow to _issue_ those wakeups delays its own message, which is correct — it chose to be low priority.

### Low-priority sender → _several_ subscribers of mixed priority

Here it gets interesting, and this is where the PI boost in `LocklessQueueWakeUpper::Wakeup()` (`aos/ipc_lib/lockless_queue.cc`) comes from.

Signals do not give an atomic, global wakeup of all watchers. They are delivered one at a time, in a loop, in the sender's context. Consider:

1. Sender, RT priority 5
2. Receiver, RT priority 7
3. Receiver, RT priority 8, which itself sends to (4)
4. Receiver, RT priority 6

The sender wakes (3) first. (3) immediately preempts the sender at priority 8, runs, and sends to (4) — which starts running at priority 6. Meanwhile (2), at priority 7, has not been signaled yet, because the sender never got back on the CPU to finish its loop. **A priority-6 task is running while a priority-7 task that is logically ready has not even been told.** That is a priority inversion with a bound set by however long (3) and (4) take.

AOS fixes this with two rules:

1. **Always wake up in priority order.** The watcher table is copied out of shared memory, sorted by priority descending, and signaled highest-first. Sorting a copy also means a watcher slot changing underneath the loop cannot corrupt the ordering.
2. **Boost the sender to the highest watcher priority for the duration of the wakeup loop, then drop back.** With the boost, the sender at effective priority 8 finishes signaling everybody before any of them run.

You can see both in the code: the copy-and-sort, then

```cpp
const int max_priority = std::max(current_priority, watcher_copy_[0].priority);
if (max_priority > current_priority && current_priority > 0) {
  SetCurrentThreadRealtimePriorityLowLevel(max_priority, SCHED_FIFO);
}
// ... signal every valid watcher, highest priority first ...
if (max_priority > current_priority && current_priority > 0) {
  SetCurrentThreadRealtimePriorityLowLevel(current_priority, SCHED_FIFO);
}
```

### What the boost actually is, and what it guarantees

The code comments call this "PI boosting", but the mechanism is not the one that name usually refers to:

**This is a self-applied priority ceiling, not kernel priority inheritance.** No lock is held, nothing is transitive, and the kernel is not tracking a donor/donee relationship. It is a plain `sched_setscheduler(0, SCHED_FIFO, max_priority)` on the sender's own thread, held across a bounded loop of `rt_tgsigqueueinfo` calls, and then undone. Real PI — as used by the PI futexes elsewhere in `aos_sync` — is a different mechanism solving a different problem.

That distinction matters because it tells you exactly what the ceiling does and does not cover. Walking the implementation:

- **The boost window is exactly the signalling loop.** Nothing else runs inside it. That is what bounds it: N `rt_tgsigqueueinfo` syscalls, no allocation, no blocking, no I/O.
- **No allocation happens on this path.** `watcher_copy_` is sized once in `LocklessQueueWakeUpper`'s constructor and `CHECK`ed for size on every call, so the copy and the `std::sort` are both in place. The wakeup path is safe under the malloc hooks.
- **The message is already published before any of this runs.** `Send()` completes the queue swap, then `Wakeup()` is called. A watcher woken first is guaranteed to see the message; so is anything polling with a fetcher, which never needed a wakeup at all.
- **Equal priority correctly gets no boost.** The condition is `max_priority > current_priority`, so a sender at 8 waking a watcher at 8 does not touch its priority. That is right rather than an oversight: under `SCHED_FIFO`, a newly runnable thread at the _same_ priority does not preempt one that is already running, so the sender still finishes its loop.
- **`SetCurrentThreadRealtimePriorityLowLevel` is used instead of the normal setter, deliberately.** The normal `SetCurrentThreadRealtimePriority` also pre-warms `UUID::Random()`, adjusts rlimits, and flips the thread's RT-mode flag and malloc hooks. None of that is wanted mid-send — the thread is already RT and must stay that way — so the boost calls the thin `sched_setscheduler` wrapper directly.
- **Both the raise and the restore are `ABSL_PCHECK`ed.** A failure to boost, or a failure to drop back, is fatal rather than silently skipped. Getting stuck at an elevated priority would be far worse than crashing.
- **`--skip_realtime_scheduler` disables both.** Under that flag no boosting happens at all, which matters when reading test results: the ordering rule still applies, the ceiling does not.

Two edges:

- **The sender's priority argument is `0` when the event loop is not running yet.** `ShmSender::DoSend` passes `event_loop()->is_running() ? runtime_realtime_priority() : 0`, so a send issued during construction or from an `OnRun` handler before the loop starts takes the non-RT path and does not boost.
- **The boost is subject to the sender's own `RLIMIT_RTPRIO`.** In the normal case this never bites: `InitRT()` pins the soft limit at 40 for unprivileged processes, and watchers are held to the same ceiling, so any priority a watcher could have registered is a priority the sender can boost to. It can bite in a mixed-privilege system — a root application registering a watcher above 40 on a channel an unprivileged RT application publishes to would make that sender's boost fail `EPERM`, and the `ABSL_PCHECK` turns that into a crash on the send path.

Note also that the boost itself is not asserted by any test: `LocklessQueueTest.WakeUpThreads` uses watcher priorities below the sender's, so it exercises the sort and the signalling loop while skipping the ceiling.

### High-priority sender → low-priority subscriber

The reverse direction is bounded by construction, and the bound is _charged to the sender_. Waking N watchers costs N syscalls in the sender's context, at the sender's priority. It does not wait for anyone. A high-priority sender publishing to a channel with many subscribers pays a cost proportional to the subscriber count on every send — a real cost, and a reason to be thoughtful about fanout on hot channels, but a bounded one that shows up directly in that sender's handler time in its timing report.

The `max_priority > current_priority` condition means no boost happens at all in this direction; the sender is already the highest priority in the group.

### Non-RT senders do not boost

The third rule, and the one to understand before relying on it: **if the sender isn't RT, there is no boosting.**

`current_priority > 0` gates the boost, and `runtime_realtime_priority()` returns 0 for anything not on `SCHED_FIFO`/`SCHED_RR`. So a `SCHED_OTHER` sender waking RT watchers does not get raised into the RT class. It still sorts and still signals highest-first — rule 1 applies regardless — it just never applies the ceiling.

The reasoning is that raising a non-RT process into the RT class is hard to bound. A non-RT process is, by definition, not holding itself to RT discipline — it may have another thread paging memory in, or blocked in the kernel on a lock. Boosting it drags all of that into the RT domain. The alternative would be defining exactly what restrictions a non-RT process must satisfy to be safely boostable, which nobody has needed enough to do.

The practical consequence: **a non-RT sender can allow an inversion between two RT subscribers of the same channel.** It signals the high-priority watcher, gets preempted before reaching the low-priority one, and the second watcher waits for a `SCHED_OTHER` thread to be scheduled again.

That is less severe than it first reads, because of [the content-free wakeup](#the-wakeup-carries-no-information). The stranded watcher is not waiting on that particular signal — it is waiting for _any_ wakeup, and the next one it gets from any source makes it scan every queue and find the message. So the real bound is whichever comes first: the non-RT sender resuming and finishing its loop, or the watcher's next timer, phased loop, or message on any other channel. For a periodic control loop that is its next period, not an unbounded wait.

What is genuinely lost is the promptness guarantee for that one message, not the message. If that matters on a channel, make the sender RT. The design's assumption is that if a process chose to be non-RT, nothing it publishes is urgent enough for the difference to matter — reasonable, but worth checking against your actual channel graph rather than inheriting silently.

### The PID/TID reuse race

Because signals target a PID/TID pair read from shared memory, there is a window between reading the watcher table and sending the signal in which the target could exit and its TID be recycled. AOS narrows this by snapshotting the robust-futex ownership word (which carries the kernel's owner-died bit) together with the PID, re-reading it after the PID/priority load, and marking any watcher whose ownership word changed as invalid.

The window is not closed. Closing it needs `pidfd`s, and opening a `pidfd` is not RT. The residual risk is an O(1 ms) window in which a signal could be delivered to an unrelated process; the mitigation is UID isolation, which the queue enforces at initialization (see the extended comment in `InitializeLocklessQueueMemory`).

`ThreadSignalSender::Signal` therefore tolerates exactly one error and no others. `ESRCH` — the target thread already exited — is the benign half of this race and is ignored. **Everything else is fatal, including `EAGAIN`**, which means `RLIMIT_SIGPENDING` was hit. That limit is a per-user aggregate across every pending realtime signal, not scoped to this signal or this target, so an unrelated process saturating it silently drops a wakeup and leaves a thread waiting forever with no record. Crashing loudly is the better failure.

---

## Layer 4: Startup and connection costs

A real-time system that only works if nothing ever starts or restarts is not useful — you restart applications during development constantly, and processes crash in the field. The same goes for looking at it: `aos_dump`, `aos_timing_report_streamer`, a freshly started logger all attach to channels exactly the way any application does, and a system you are debugging is precisely the one where the diagnostic tool must not cost anyone a deadline.

The whole strategy is: **all of the expensive, unbounded, lock-taking work is done at attach time, before the process goes RT, and every RT path is designed so that a process attaching concurrently cannot disturb it.**

### Attach is under a lock, and that lock is non-RT

Attaching to a channel means opening its shared memory file (one per channel, sized purely from the config so every process computes the same number), mapping it in — writable plus a read-only mapping for what gets handed to application code — and touching every page so no later RT access takes a fault. `MemoryMappedQueue` (`aos/ipc_lib/memory_mapped_queue.cc`) does all of that, and then initializes the queue under the setup lock.

That lock, `GrabQueueSetupLockOrDie`, is a robust mutex living in the queue's own shared memory. `LocklessQueueWatcher`'s constructor takes it too, and then linearly scans the watcher table for a slot that is either unclaimed or whose owner is definitely dead.

Taking a lock here is fine precisely because nobody on an RT path takes it. Sends and reads never touch it. The lock only serializes _attaching_, so an attach can be as slow as it needs to be — an O(n²) cleanup algorithm is fine when there are seconds available.

### Cleanup is deferred to the next attacher

When a process dies mid-send it may leak a message reference. Nobody cleans that up promptly, and nothing tries to. Instead, the next process to register as a sender or watcher does the cleanup, under the setup lock: walk the entire queue and the sender scratch slots, work out which messages are unaccounted for, and repair the affected senders.

This is a deliberate trade. The alternative — cleaning up promptly — would mean putting recovery logic on a path that RT code touches. Deferring it means the cost of a crash is paid by the _next restart_, in non-RT setup code, where it costs nothing that matters. The bound is that leaked references cannot grow without limit, and they can't: each dead process leaks at most one, and restarting it reclaims it.

### The `Construct()` / `Startup()` split

`ShmEventLoop::Run()` splits watcher setup deliberately:

```cpp
// Non-RT: grab watcher slots, clean up after dead processes, start queueing
// signals.
for (auto &watcher : watchers_) { watcher->Construct(); }

// ... lock memory, go RT ...

// RT: snap each watcher's queue index to the newest message.
for (auto &watcher : watchers_) { watcher->Startup(); }
```

`Construct()` is the expensive half — locks, scans, cleanup — and runs on `SCHED_OTHER`. Once it returns, signals are already being queued for this process, so no message sent from this point on is lost; the cost is a small amount of extra signal processing during the remaining startup.

`Startup()` runs _after_ the transition to RT, and does nothing but snap the read pointer to the newest message. Putting it on the RT side minimizes the window between "we decided where to start reading" and "we are actually able to respond", which is the window in which messages get dropped on the floor at startup.

### Adding a subscriber does not disturb RT traffic

A sender's wakeup loop reads the watcher table without any lock. A watcher being added concurrently is either fully committed (its ownership word is set — which happens _last_, after PID and priority, with a barrier) or invisible. There is no state in which a half-registered watcher can be signaled or can corrupt the sort.

This is what delivers the promise from the top of this layer: pointing a diagnostic tool at a live system, or restarting the logger, does not perturb the control loops.

### What is still not free

Honest accounting of what attaching _does_ cost:

- The setup lock is shared per channel. Many processes starting simultaneously serialize on it. This is a startup-time throughput issue, not an RT issue.
- Pre-faulting means attach time scales with the total size of every queue the process maps, not with how much of it gets used. That is the intended trade — the alternative is paying it as faults, later, on the RT path.
- `Aio`/io_uring timer creation touches the ring, so `--aio_queue_depth` must be at least the concurrent timer count. It fails loudly at construction rather than subtly later.

---

## Layer 5: The event loop

`ShmEventLoop` is single-threaded by design. The reasoning is that threads are hard, multithreaded programs are less repeatable, and splitting work across _processes_ instead gets you partial restarts, crash isolation, and enforced use of the IPC layer for all communication — all of which are good things independently.

### Split RT and non-RT work across processes, not threads

That preference is the single highest-leverage decision available when structuring an application, so it deserves stating as a rule:

> **Put real-time work and non-real-time work in separate processes wherever you can.**

Threads are the tempting answer — the data is right there, no serialization, no config entry. But a thread shares an address space, and an address space is not something you can hold to a priority. A non-RT sibling that allocates or frees fires TLB shootdown IPIs at your RT thread's core and contends for `mmap_lock` with it, and neither of those respects any priority you set. The same goes for the allocator arena, the descriptor table, and the page-fault path. Those are not bugs to be tuned around; they are what sharing an address space means. See [Interference from inside your own process](measuring_realtime.md#interference-from-inside-your-own-process) for the full list and how it shows up in a trace.

A separate process has none of it. Different `mm_struct`, so no shared `mmap_lock` and no shootdown IPIs; different allocator, different fd table. The isolation is structural rather than something you maintain by being careful.

Linux itself does not really have a thread/process distinction to argue about. Both are tasks created by `clone()`; what differs is only which structures get shared. Linus's framing, back in the early Linux threading discussions, was that "traditional threads suck" and that the goal was for threads to be "just another form of processes" — the interesting question being _what you share_, not which of two categories you picked. Taking that seriously is what this rule amounts to: a thread shares the address space, the descriptor table, and the signal disposition whether you wanted those or not, and each of them is a documented way for non-RT work to reach an RT thread. Choose the sharing deliberately, and for RT versus non-RT the right amount to share is a channel.

**And AOS makes the split cheap.** The usual objection is that you now have to move data between the halves; here that is a channel — a lockless queue with a zero-copy flatbuffer in shared memory, timestamped and ordered, with the wakeup inversions already handled (Layers 2 and 3).

The data movement is a pointer swap, but **the real cost is the context switch**, which on modern hardware dominates: the scheduler trip, plus the cache and TLB damage both sides keep paying afterwards. Two things make that acceptable. If you were reaching for a thread you had already accepted a switch — a process only adds an address-space change, which does not flush a tagged TLB (PCID, ASID). And a loop that [fetches its inputs](#fetchers-exist-for-a-real-time-reason) wakes once per cycle regardless of how many processes feed it. So split where the isolation is worth a wakeup, and not so finely that you spend the budget switching.

So: put the control loop in one application at RT priority, put the logging, the diagnostics, the network chatter, and the vision pipeline in others, and let the channels connect them. When you genuinely do need threads inside an RT application, [Threading](threading.md) covers configuring them — but reach for that second.

### Everything is a file descriptor

Everything the loop waits on is a file descriptor:

- Message wakeups: the `signalfd` receiving `kWakeupSignal`.
- Timers: `timerfd`s. `Schedule()` is a single `timerfd_settime(2)` with an absolute `CLOCK_MONOTONIC` deadline; `Cancel()` is the same call with a zero `it_value`. Neither allocates, and neither has an asynchronous tail to reconcile.
- Everything else: sockets, CAN, V4L2, whatever, via the legacy readiness API.

On Linux the backend is `io_uring` by default, configured `IORING_SETUP_SINGLE_ISSUER | IORING_SETUP_DEFER_TASKRUN` so the kernel only processes completion work when the application explicitly asks, rather than opportunistically at the end of arbitrary syscalls. `--aio_backend=epoll` selects the fallback. The full rationale, including the same-thread destruction constraint it interacts with, is in `documentation/adr/0001-aio-io-uring-single-issuer.md`.

Because an absolute deadline is handed straight to the kernel, a periodic timer that re-arms against a grid it owns — which is what `ShmTimerHandler` and `PhasedLoop` do — never accumulates phase error, no matter how late any individual firing is. A busy loop delays a firing; it does not shift the grid.

### Fetchers exist for a real-time reason

A lot of the messages an application receives don't trigger any action — they get copied and saved for a fixed-frequency control loop to use later. Watching those channels turns each one into a wakeup anyway: a signal delivery, a context switch — with all the scheduler, cache, and TLB costs described above — and a callback that does nothing but save a pointer. All of it to arrive somewhere the loop was going to get to on its own schedule regardless.

`Fetcher` removes the wakeup instead of optimizing it. `Fetch()` grabs the latest message, `FetchNext()` walks forward one at a time, and neither needs anyone to have signaled anything. A control loop that runs on a timer and fetches its inputs wakes up exactly once per cycle no matter how many channels it consumes — one wakeup instead of one per incoming message across N channels.

That is a jitter win, since the loop's wakeups are now its own timer rather than whatever traffic happened to arrive. On a box running many applications it is a larger CPU win, and CPU is the scarcer resource: those context switches were buying nothing. Hence the idiom most AOS control loops should follow — watch the channel that should actually make you act, and fetch everything else.

---

## Layer 6: The system around the application

An application can be perfectly RT and still miss deadlines because something else on the box stole the CPU. AOS ships tooling for the rest of the system.

### IRQs and kernel threads

This is the part of the problem the user of an RT system usually discovers last and most painfully. On `PREEMPT_RT`, most interrupt handlers run as kernel threads (`irq/N-name`), which means they are schedulable — and by default they are scheduled at priority 50, above most application threads. (A stock kernel gets the same threads from the `threadirqs` boot parameter — see [Running without `PREEMPT_RT`](#running-without-preempt_rt).) A burst of network or MMC interrupts will preempt your control loop.

`aos/starter/irq_affinity.cc` fixes this from configuration (`aos/starter/kthread.fbs`):

```json
{
  "irqs": [
    {"name": "eth0", "affinity": [0]},
    {"name": "mmc0", "affinity": [0]}
  ],
  "kthreads": [
    {
      "name": "irq/*-e0002000",
      "scheduler": "SCHEDULER_FIFO",
      "priority": 45,
      "affinity": [1]
    },
    {"name": "irq/*-s-mmc0", "scheduler": "SCHEDULER_OTHER", "nice": -20}
  ],
  "threads": [
    {"name": "FRC_NetCommDaem", "scheduler": "SCHEDULER_FIFO", "priority": 25}
  ]
}
```

Three separate levers, and they do different jobs:

- **`irqs`** writes `/proc/irq/N/smp_affinity`, steering the _hardware_ interrupt to specific cores. Use this to keep interrupt traffic off the cores running control loops.
- **`kthreads`** sets scheduler policy, priority, and affinity for kernel threads by glob (matching the `irq/*-name` threads the RT patchset creates, plus softirq threads like `ksoftirqd`). Use this to put IRQ threads _below_ your critical loops, or to demote a chatty one to `SCHED_OTHER` with a negative nice.
- **`threads`** does the same for non-AOS userspace threads you don't control — vendor daemons, for example.

`irq_affinity` re-applies configuration as threads appear, so it keeps working across driver load and process restart. `aos/starter/roborio_irq_config.json` is a worked example.

The combination this design exists to prevent: a hardware IRQ raises a softirq, the softirq thread runs at priority 50, your control loop at priority 33 waits behind it, and the delay is bounded only by how much work the driver decides to do. Explicit priorities and affinity turn that into a bound you chose.

#### Priority is not a substitute for affinity

Prioritizing IRQ threads handles the _deferred_ half of interrupt work. It does nothing about the hard IRQ handler, which runs when the interrupt fires regardless of what your threads are doing — short, but not free, and not something your priorities have any say over. Steering the interrupt to another core with `irqs` is the only thing that removes that cost from a core running a control loop. The two levers solve different halves of the same problem, and you generally want both.

#### NAPI is per-core, not per-device

Network receive processing gets its own section because the obvious mental model is wrong in a way that matters. NAPI polling is driven from a `poll_list` in the per-CPU `softnet_data`: `net_rx_action` walks whatever NAPI instances got scheduled on _this core_ and polls them in sequence, against a shared budget. It is a per-core loop, not a per-device one.

So two devices whose interrupts land on the same core share one poll loop, and there is no priority between them inside it. A latency-critical CAN interface and a busy ethernet port on the same core will interleave, and the CAN traffic waits behind however much ethernet work the budget allows — no matter what priority you gave the IRQ threads, because that ordering is decided inside a single softirq's poll loop rather than by the scheduler.

Two ways out, and they compose:

- **Steer the interrupts to different cores** with `irqs`, so the two devices are never on the same poll list.
- **Turn on threaded NAPI** (`echo 1 > /sys/class/net/<dev>/threaded`), which gives each NAPI instance its own kernel thread named `napi/<dev>-<id>`. Once the polling is a thread rather than softirq context, it has a PID, and priority and affinity become yours to set — per device, which is exactly the control the per-core poll loop denies you. Those threads match the `kthreads` glob like any other.

  Note the floor: the `threaded` sysfs control landed in **Linux 5.12** (`5fdd2f0e5c64`, Feb 2021). It is well inside the 6.1 floor `Aio` already requires, but it is recent enough that it won't exist on an older vendor kernel, and the failure is a missing sysfs file rather than an error message that explains itself.

### Starter

`starterd` launches applications with the priority, affinity, user, and cgroup limits from the config. It also emits an ftrace marker and stops tracing on `SIGCHLD` when `--enable_ftrace` is set, so an unexpected application death freezes the trace buffer at the moment of death.

### cgroups

Memory limits per application via `memory_limit` in the config. See [Cgroups](cgroups.md). These are a _containment_ mechanism rather than an RT one: a process that hits its limit is killed rather than being allowed to swap, which would cost everyone else latency.

---

## Running without `PREEMPT_RT`

Everything above assumed `PREEMPT_RT`, and the hard bound in [What "real-time" means here](#what-real-time-means-here) genuinely needs it. But sometimes you don't get a choice — a vendor kernel that won't take the patch, a cloud machine, a desktop — and it is still useful to run AOS as if it were real-time. Everything AOS does in userspace works unchanged: the queue, the malloc hooks, `mlockall`, the wakeup ordering and boost, the PI futexes (mainline has had `FUTEX_LOCK_PI` for decades), `SCHED_FIFO` itself. What changes is the kernel underneath:

|  | `PREEMPT_RT` | Stock kernel |
| --- | --- | --- |
| Kernel spinlock sections | sleeping locks with PI, preemptible | preemption disabled — your core is taken until unlock |
| Kernel sleeping locks | priority inheritance | no PI — a kernel-lock inversion is unbounded |
| Hard IRQ handlers | threads (`irq/N-*`), schedulable | interrupt context — priorities have no say |
| Softirqs | preemptible task context | run at IRQ exit, on top of whatever was running |
| Waking an RT thread | bounded, small | bounded by whatever kernel code your core is in |

Most of those rows are latency you measure and accept. The softirq row changes behavior, and it is the one to understand first.

**Softirqs get vicious on a stock kernel.** On `PREEMPT_RT`, softirq work always runs in somebody's task context, where your priority protects you. On mainline, a softirq raised by a hard interrupt runs _at interrupt exit, on the CPU that took the interrupt, on top of whatever was running there_ — your control loop included — and keeps running for up to ~2 ms and ten restarts (`MAX_SOFTIRQ_TIME`, `MAX_SOFTIRQ_RESTART`) before the kernel gives up and defers the remainder to `ksoftirqd`. No priority helps, because none of this goes through the scheduler. A burst of network traffic on the wrong core is milliseconds stolen from a handler mid-flight, visible in the timing report only as handler time nothing preempted.

Two levers recover most of it — pinning and threaded IRQs get you surprisingly far:

- **Boot with `threadirqs`.** This mainline boot parameter forces interrupt handlers into `irq/N-*` threads, exactly like `PREEMPT_RT` does, so the `kthreads` config from [IRQs and kernel threads](#irqs-and-kernel-threads) applies unchanged. It also relocates the softirqs: one raised inside a force-threaded handler runs at the end of that handler, in that thread; one raised in remaining hard-IRQ context is deferred to `ksoftirqd` instead of running at IRQ exit. Either way the work lands in a thread you can prioritize and pin. Give `ksoftirqd` an explicit priority below your control loops — and note that an RT thread that spins can now starve it entirely on that core, which is one more reason the [RT throttle](measuring_realtime.md#rt-throttling) stays on. A few handlers marked `IRQF_NO_THREAD` (timers, per-CPU interrupts) stay hard regardless.
- **Steer interrupts harder than you would on RT.** Both the hard handler and the softirq it raises run on the CPU the interrupt landed on, so moving an interrupt off the RT core with the `irqs` config removes both at once. On `PREEMPT_RT`, affinity and priority are two levers that compose; on a stock kernel, affinity is the load-bearing one, and the advice in [Affinity and pinning](measuring_realtime.md#affinity-and-pinning) shifts from "pin the minimum" toward "steer everything interrupt-shaped away from the loop". [Threaded NAPI](#napi-is-per-core-not-per-device) works on mainline too.

Also take the most preemptible kernel you can get: `CONFIG_PREEMPT`, or `preempt=full` on a `PREEMPT_DYNAMIC` kernel, narrows the last row of the table to spinlock and irqs-off sections — the difference between millisecond and tens-of-millisecond tails.

What you end up with is a system whose average behavior looks like the RT one, with a tail set by the longest spinlock section or unthreadable interrupt your workload can hit: typically hundreds of microseconds to a few milliseconds on a well-configured machine, and unbounded in principle. Measure it with `cyclictest` ([Rung 0](measuring_realtime.md#rung-0-establish-the-floor)) and hold it against the deadline you actually have. For a lot of systems that is enough; just don't mistake it for the bound this document started with.

---

## What AOS does not do for you

Being explicit about the edges:

- **It does not make your algorithm bounded.** An unbounded loop in a message handler is still unbounded. Timing reports will show it; nothing prevents it.
- **It does not protect an RT thread from a non-RT sender's inversions**, by design (see above).
- **It does not audit your priority assignment.** Getting rate-monotonic ordering wrong is a design error AOS will faithfully execute.
- **It does not fix the kernel.** If your kernel has a driver with a 5 ms spinlock section, no amount of userspace design helps. `cyclictest` is the right tool to establish the floor before blaming AOS; [Running without `PREEMPT_RT`](#running-without-preempt_rt) is how far you can raise it when the RT patch isn't an option.
- **It does not cover the network.** Message bridge moves messages between nodes over SCTP; that path has its own latency characteristics and its own timing instrumentation, but the guarantees above are about a single node's shared memory.

---

## Further reading

- [Measuring Real-Time Performance](measuring_realtime.md) — how to verify all of the above on hardware
- [Threading](threading.md) — worker threads inside an application
- [Cgroups](cgroups.md) — memory containment
- `documentation/adr/0001-aio-io-uring-single-issuer.md` — the `io_uring` backend and its threading constraints
- `aos/ipc_lib/lockless_queue.h` — the queue API and its guarantees
- `aos/realtime.h` — the RT-mode API
