# ADR 0002: kqueue backend for Aio on macOS — restoring the ordering and readiness semantics the kernel does not provide

## Status

Proposed. Implemented in `aos/events/aio_darwin.cc`, `aos/events/timer_queue.h`, `aos/events/aio_uv.cc` and `aos/events/glib_main_loop.cc`. The kqueue backend, the libuv backend on macOS, and `GlibMainLoop` on macOS are all in tree and tested: `//aos/...` runs 135 test targets green on `darwin_arm64`, and `//...` 212. Twenty targets remain Linux-only for reasons unrelated to `Aio` (SCTP, `signalfd`, `timerfd`, `starter`/subprocess, stacktraces), and ten cases inside `aio_test` skip themselves as io_uring-specific.

## Context

`Aio` (`aos/events/aio.h`) is AOS's cross-platform completion-based async I/O primitive; ADR 0001 covers its io_uring and epoll backends on Linux. `ShmEventLoop` and `Epoll` are built on it, so bringing macOS to parity means a third backend rather than a second event loop.

The temptation with a third backend is to treat whatever the platform hands back as the contract. That does not work here, because `Aio`'s contract was written against Linux and callers already depend on parts of it that kqueue does not provide. Three of those are load-bearing:

1. **`Poll()` delivers exactly one completion per call**, so a callback runs while the rest of a due batch is still pending and can be cancelled, rescheduled or destroyed. This is what `Aio::Timer`'s destructor does when a timer callback drops its own timer.
2. **Timers due at the same instant are delivered in the order they were scheduled.** Both Linux backends get this from the kernel for free: each timer is its own `timerfd`, and epoll/io_uring report them in the order the kernel made them ready. Callers rely on it — a callback for the earlier deadline may act on a later timer whose firing is already queued.
3. **A readiness bit is only ever reported if it was subscribed to**, and error conditions are reported regardless of the mask. `epoll_ctl(2)` guarantees the first half directly.

kqueue provides none of these as given, and the ways it diverges are not symmetrical to anything on Linux:

- Its ready list is **a stack**. Two `EVFILT_TIMER` knotes armed for one absolute deadline come back most-recently-armed first — exactly backwards from (2). Verified directly, reproducibly, on macOS 25.6/arm64.
- It has **no error-only filter**. Watching for errors on the write side means arming a whole `EVFILT_WRITE`, which then also reports writability nobody asked for, violating (3).
- It has **no separate error indication at all**. `EV_EOF` is the only terminal signal, and it means different things depending on which filter reported it.
- A kqueue descriptor is **not inherited across `fork(2)`**, where an epoll fd and a timerfd both are.
- A thread wakeup **cannot be addressed to a thread**. `ThreadSignalSender::Signal(pid, tid)` is `tgkill(2)` on Linux, waking exactly the loop that owns the target's `signalfd`; macOS has no equivalent, so `thread_signal_darwin.cc` falls back to `kill(2)` on the process. `EVFILT_SIGNAL` then fires on _every_ kqueue in the process watching that signal.

macOS also lacks `timerfd` and `signalfd` outright, which is what had previously kept the libuv backend (`aos/events/aio_uv.cc`) and `GlibMainLoop` Linux-only.

## Decision

Treat the Linux semantics as the contract and make the kqueue backend meet it, rather than letting the platform's behaviour leak into `Aio`'s API. Concretely:

1. **Timer dispatch order is maintained in userspace, not asked of the kernel.** `TimerQueue` (`aos/events/timer_queue.h`) keeps armed timers ordered by `(deadline, sequence)`; `front()` is both the deadline to wait for and the timer to dispatch when it arrives.
2. **One `EVFILT_TIMER` knote exists at a time**, armed for `front()`'s deadline — not one knote per timer. `KqueueImpl::UpdateTimerKnote()` is the only thing that touches it, and every path that can change the front calls it.
3. **`EV_EOF` is translated against the filter that reported it**, because kqueue uses one flag for two conditions epoll reports separately. On `EVFILT_READ` it means the writer is gone and reads return a clean end-of-file, which is `EPOLLHUP` -- not an error, and paired with `EPOLLIN` so the reader drains. On `EVFILT_WRITE` it means the _reader_ is gone and every subsequent `write()` fails with `EPIPE`, which is `EPOLLERR`. Folding both into the error bit aborts an ordinary `OnReadable()`-only registration on the `err_fn` CHECK; dropping both leaves `OnError()` with nothing that can raise it.
4. **Readiness the caller did not subscribe to is masked off** before dispatch, so `events` always holds the mask epoll would have produced and the dispatch below it can stay a transcription of the Linux backend.
5. **A kqueue is itself a descriptor**, so one holding a single `EVFILT_TIMER` or `EVFILT_SIGNAL` stands in for the `timerfd` and `signalfd` that libuv needs and macOS does not have.
6. **`Aio`'s event encoding is treated as its own**, not as epoll's, so consumers translating it need no Linux-only header.
7. **Wakeup addressing is the one divergence left standing.** A wakeup meant for one loop wakes every loop in the process. `thread_signal.h` already documents wakeups as possibly spurious, so this is inside the contract rather than a violation of it -- see the Consequences.

## Design

The backend sits behind the shared `internal::ReadinessBackend` (`aos/events/aio_readiness_backend.h`), which owns everything epoll and kqueue do identically — the registration table, the raw request machinery, the drain loop, the fork check, and the dispatch rules. What is left below is the seam: how interest is expressed to the kernel, and how the kernel answers. Everything in this section lives in that seam.

The IOCP backend does not sit behind it. `aio_windows.cc` names neither `ReadinessBackend` nor `FdRegistrationTable`: it is completion-based rather than readiness-based, so there is no "which of these descriptors are ready" question for the seam to ask, and it keeps its own registration pool because a completion-based backend must park a registration while the kernel still holds a pointer into it — a lifetime the readiness table has no notion of. ADR 0003 covers that backend and what the shared table would need before it could absorb it.

Timers and signals reach it through `WaitResult::kHandled`, which exists because kqueue delivers them as filters of their own where epoll delivers them as ordinary descriptors that fall out of the shared path.

### Timer ordering: `TimerQueue`, keyed on `(deadline, sequence)`

`sequence` is a counter `TimerQueue::Insert()` stamps on each arming. It does two jobs: it makes equal deadlines FIFO instead of arbitrary, and it makes the order **total**, so no two timers ever compare equal and the tree needs no duplicate handling. Stamping it inside `Insert()` rather than letting the caller supply it means the FIFO rule cannot be got wrong from outside.

The structure is an intrusive red-black tree: O(log n) to insert and remove, O(1) to ask what is next (`min_` is cached), with no case that degrades. Intrusive and allocation-free, so arming a timer on an RT thread allocates nothing.

A sorted intrusive list was tried first and is the more obvious choice, but it is O(1) only at whichever end it searches from. Searching from the front makes re-arming a periodic timer — the operation that runs once per firing — walk every armed timer; measured at ~N link traversals per re-arm, 288 of them with 256 timers armed. Searching from the back fixes that and moves the linear case to inserting a timer sooner than everything armed. A tree has no bad end to search from.

The tree's links live on `KqueueTimerState`, not on `Aio::TimerState`, because `prev_active`/`next_active` there are the Linux backends' intrusive list and a timer is only ever on one backend's structure.

### One knote per wakeup, not one per timer

The obvious shape is one `EVFILT_TIMER` knote per `Aio::Timer`, keyed by its `TimerState` — it is what the Linux backends do with `timerfd`, and it makes the kernel the thing that decides which timer is due. On kqueue it does not work, and the reason it cannot be patched up is the interesting part.

**The kernel's choice is wrong.** With several knotes due at the same instant, kqueue reports them most-recently-armed first, because its ready list is a stack. That is the reverse of the order they were scheduled in, which is the order callers depend on.

**It cannot be corrected after the fact.** By the time an event arrives the kernel has already chosen, and — this is the part that closes the door — the knotes it did not choose _have not fired_. There is no batch to sort. `kevent()` hands back one event, and the others are still armed in the kernel, indistinguishable from timers that are not due at all. Restoring the order therefore means re-deriving "which armed timer is actually earliest" from userspace state on every single firing, and then re-arming the one the kernel offered if it lost — which is an extra syscall, because its `EV_ONESHOT` knote was consumed by being reported. That is O(armed timers) plus a syscall per firing, and firing is the operation that happens often; arming is the rare one. An earlier revision did exactly this and was removed for that cost.

**So the choice has to be taken away from the kernel.** If only one deadline is ever waited on, there is nothing for the kernel to choose between: the wakeup means "the earliest deadline has arrived", and which timer that is comes from `active_timers_`. The ordering problem stops existing rather than being corrected, and it costs one knote instead of n.

The cost of being wrong here is not theoretical. It is what `OneCompletionPerPollTest` and the three same-batch tests caught, and what a per-timer arrangement passed on Linux and failed on macOS.

`UpdateTimerKnote()` arms a single `EVFILT_TIMER` at `kTimerIdent` for `front()`'s deadline, with `NOTE_MACHTIME | NOTE_ABSOLUTE` so the nanosecond deadline survives (`uv_timer_t`'s milliseconds would not, which is the same trade the Linux backends make with `timerfd` over io_uring timeout ops). `EV_ONESHOT`, so a firing disarms it; `timer_armed_`/`armed_deadline_` skip the syscall when the front deadline has not moved.

The knote carries no identity. When it fires, the timer to dispatch is `front()` — which is what makes the one-completion-per-`Poll()` contract and the ordering contract the same mechanism rather than two. Dispatch takes the front, resolves it (`request.done`, remove from the queue) _before_ running the callback so the callback may destroy it, and calls `UpdateTimerKnote()` _after_ the callback so that whatever the callback did to the timers is what gets armed. A deadline already in the past re-arms to fire immediately, which is how the rest of a due batch is delivered one `Poll()` at a time.

Dispatch is guarded on the timer actually being due. The front can move to a later deadline between the kernel queueing the event and `Poll()` reading it — a cancel, or a reschedule — and the re-arm is what waits for the new one. This cannot spin: the re-arm is for a future deadline, so the next blocking `kevent()` sleeps until then.

`ToMachTicks()` rounds up. Truncating let a knote fire up to one tick before its deadline, bounce off that dueness guard, and re-arm — an extra `kevent()` round trip per firing, for nothing.

### `EV_EOF` means two different things

kqueue gives no separate error bit, so `EV_EOF` has to be read against the filter that reported it:

- **On the write filter** the reader is gone and every future `write(2)` fails with `EPIPE`. That is precisely what epoll reports as `EPOLLERR`, so it translates to the error bit. This is the pipe-with-a-closed-read-end case, and dropping it leaves `OnError()` silent on Darwin with nothing else to raise it.
- **On the read filter** the writer is gone and reads return a clean EOF. That is `EPOLLHUP` — not an error, and epoll pairs it with `EPOLLIN` so the reader drains. Folding this one into the error bit instead aborts an ordinary `OnReadable()`-only registration on the `err_fn` CHECK.

Both directions are pinned by tests: `EPollLikeBasicError` for the write side, `LegacyReadableIgnoresHangup` for the read side.

### Readiness masking

`DesiredReadiness()` is the single definition of what a registration subscribed to, used both by `UpdateKqueue()` to decide which filters to arm and by `Poll()` to mask what comes back. The bug it exists to prevent is those two disagreeing.

Masking cannot hide a filter that still needs draining, by construction: `UpdateKqueue()` arms a filter `EV_CLEAR` exactly when its readiness bit is not desired, so a bit dropped at dispatch always belongs to an edge-triggered filter that will not re-report it.

### A wakeup outlives the receiver that failed to consume it

`aio.h` promises that once `UnregisterThreadSignalReceiver()` returns, a wakeup still kernel-side belongs to whoever registers next. The signalfd backends get that by doing nothing: an unregistered `signalfd` simply stops being polled and keeps whatever is queued in it. kqueue has no such property to inherit — `EVFILT_SIGNAL`'s count lives on the knote, `EV_DELETE` destroys it, and because `kWakeupSignal` is `SIG_IGN` here rather than blocked there is no process-pending copy to fall back on.

So the knote is **not** deleted on unregister. Unregistering clears the callback and leaves the kernel watching; the knote dies with the kqueue in `~KqueueImpl()`. Re-arming for a successor is `EV_ADD` on the live knote, which updates it in place and preserves the count.

That alone is not enough, and the gap is easy to miss: leaving the knote armed means the loop can _observe_ a wakeup with nobody registered to take it — unregistering does not stop the loop — and `EVFILT_SIGNAL`'s count is consumed by being reported. Noticing it there would lose it just as thoroughly as deleting the knote. It is therefore parked in `signal_wakeup_pending_`, the same slot the post-fork replay already used for a wakeup that has nowhere to go yet, generalized to mean exactly that.

The cost is that a loop which has unregistered but is still polling now wakes once per process-directed signal, where before its knote was gone. Bounded to one iteration per signal — `EVFILT_SIGNAL` is implicitly `EV_CLEAR` and reports a count, so a hundred signals coalesce into one event — and narrow in practice, since `ShmEventLoop` only unregisters during shutdown.

One property this leans on, which is worth stating because the intuition runs the other way: **a wakeup here is a broadcast, not a queue.** Every armed knote in the process gets its own count, so a loop consuming its own takes nothing from anyone else's. That is the same fact as the fan-out above, seen from the other side — and it is why an unregistered-but-armed loop cannot starve a registered one. Measured: a loop spinning on an unregistered knote consumed 300 of 300 signals while another still received all 300.

### libuv on macOS: a kqueue is a descriptor

libuv can only wait on descriptors, so each thing AOS needs waking for has to be one. Linux already has a purpose-built descriptor for each — a `timerfd` per timer, and the receiver's own `signalfd`. macOS has neither, but a kqueue goes readable when one of its filters fires and nests inside the kqueue libuv polls with (verified against `EVFILT_READ`, which is how `uv_poll` watches an fd there). So a kqueue holding one `EVFILT_TIMER` stands in for the timerfd, and one holding one `EVFILT_SIGNAL` stands in for the signalfd that `ThreadSignalReceiver` does not have on this platform — it only sets `kWakeupSignal` to ignored and leaves observing it to whoever cares.

The difference is confined to six helpers behind one `#ifdef` (`CreateDeadlineFd`, `ArmDeadline`, `DisarmDeadline`, `DrainDeadline`, `CreateWakeupFd`, `DrainWakeupFd`, plus `kOwnsWakeupFd` for who closes it); below them there is only ever an fd.

**A borrowed libuv loop does not survive `fork(2)` on macOS.** The loop's backend is a kqueue, which the child does not inherit, so every call into it there fails `EBADF`. The native backend rebuilds its own kqueue in the forked child; a borrowed loop is not `UvAio`'s to rebuild. This is a real limitation, documented in `aos/events/aio_uv.h`, and the reason `shm_event_loop_test_uv` runs its death tests `threadsafe` on this platform — a forked child inherits a dead loop and dies of that rather than of what the test came to check.

### `Aio`'s event encoding is not epoll's

`Aio::OnEvents()` documents its encoding as "the low four epoll bits on every platform": readable `0x01`, priority `0x02`, writable `0x04`, error `0x08`, with a hangup delivered as the error bit. `GlibMainLoop` was reaching into `<sys/epoll.h>` for those constants, which is the only thing that had kept it Linux-only. Naming them directly (`kAioIn`/`kAioPri`/`kAioOut`/`kAioErr` in `aos/events/glib_main_loop.cc`) removes the header and the platform gate.

Doing so exposed two translations that were already wrong on their own terms: `EPOLLRDHUP|EPOLLHUP` inbound was unreachable, since `Aio` folds a hangup into the error bit and never sets `EPOLLHUP` itself; and `G_IO_HUP` outbound became `EPOLLHUP`, a bit outside the documented four, which epoll ignores (it reports hangups regardless of the mask) but which asks the kqueue backend for no filters at all.

## Consequences

- Timer dispatch order is now a property of `Aio`, not of the platform, and is the same on all three backends. That is a stronger contract than Linux was previously relying on by accident of `timerfd` fd-number ordering.
- Arming a timer is O(log n) rather than O(1). At the scale an event loop actually runs — tens of timers — this is not measurable, and it removes the linear case that a list has at one end whichever end that is.
- The macOS backend carries userspace state (`TimerQueue`) that the Linux backends do not, because the Linux backends get the same ordering from the kernel. This is deliberate: the alternative is a weaker contract on every platform.
- `Aio::TimerState` gained nothing; the tree links live on the Darwin subclass. The Linux backends are untouched by any of this.
- **A wakeup on macOS wakes every event loop in the process, not the one it was addressed to.** With N loops, one wakeup costs N. Nothing is lost -- each loop rechecks its own state and finds nothing, which is what `thread_signal.h` means by a spurious wakeup, and `ShmEventLoop` already rechecks its channels -- but a consumer that reads "my receiver callback ran" as "there is work for me" is correct on Linux and wrong here. On an RT thread the cost is latency, not just cycles.

  Narrowing it is possible but does not belong in the backend: `KqueueImpl` only ever sees its own loop, so it cannot tell whether a wakeup was addressed to it. The process-wide view already exists in `WakeupSignalDisposition` (`aos/ipc_lib/thread_signal_darwin.cc`), which holds the receiver count and the `SIG_IGN` disposition; a target set recorded there by `Signal()` before the `kill(2)`, and consulted by each woken receiver, would turn the broadcast into a routed wakeup with the spurious ones suppressed. Not done, because nothing in tree runs two loops in one process yet and an unused mechanism is worse than a documented cost. `AioTest.TwoLoopsOnTwoThreadsTest` pins the behaviour either way: it requires each loop to see its own wakeup, not to see only its own.

- `GlibMainLoop`, `UvAio` and `shm_event_loop_test_uv` are no longer Linux-only. `aio_uv` and its tests remain incompatible with Windows, where libuv's loop is an IOCP that nothing else can wait on.

## Alternatives considered

- **Let kqueue's order stand and weaken the contract.** Rejected: three existing tests (`CancelATimerFiringInTheSameBatchTest`, `RescheduleATimerFiringInTheSameBatchTest`, `DestroyATimerFiringInTheSameBatchTest`) describe behaviour callers depend on, and `Aio::Timer`'s destructor is one of those callers. A contract that differs per platform is one that will be depended on per platform.
- **One knote per timer, with the order re-derived on every firing.** What an earlier revision did; see "One knote per wakeup, not one per timer" above for why the correction is more expensive than avoiding the problem.
- **A hashed timing wheel.** Its O(1) comes from a periodic tick advancing a cursor and expiring a bucket; this design is tickless, arming one wakeup for the next deadline and sleeping until it, so there is no cursor to advance. Adding a tick means waking an RT event loop that should be asleep. A wheel also quantizes deadlines to its tick and bounds its horizon, and AOS deadlines are absolute and nanosecond-exact.
- **A binary heap.** The right answer if the timer count ever grew: O(1) peek-min, O(log n) insert and remove-by-handle, and tickless-friendly. Not taken because a heap is not stable (equal deadlines would need the sequence number anyway), removal by handle needs an index maintained on the node, and at the scale in question it does not beat a couple of pointer chases. The costs are documented in `timer_queue.h` so the tradeoff can be revisited against evidence.
- **Routing wakeups on macOS by filtering in the receiver.** Deferred rather than rejected; see the Consequences for the shape it would take and why it is not built yet. It cannot be done in `KqueueImpl`, which is why it is not simply an oversight there.
- **Blocking `kWakeupSignal` instead of ignoring it**, to get the successor-wakeup guarantee the way Linux gets it. This is the closest structural analogue and does not work. Blocking does make the signal process-pending exactly as on Linux — but `EVFILT_SIGNAL` never consults the pending set, so a knote added afterwards reports nothing, and the successor sees no wakeup. It is also the wrong shape twice over: `sigprocmask` is per-thread while this platform's sender is a process-directed `kill(2)`, so every thread would have to block it and any that missed would take the default action and kill the process. That is why the disposition is a process-wide `SIG_IGN` to begin with. Reproducing signalfd would additionally require draining the pending set at registration (`sigtimedwait` with a zero timeout), with the double-count hazard that `EVFILT_SIGNAL` may report the same signal too.
- **`uv_timer_t` for the libuv backend's timers.** Rejected for the same reason the Linux backend rejects io_uring timeout ops: it takes milliseconds, and would quietly coarsen every AOS timer.

## Verification

- `//aos/events:timer_queue_test` covers the queue with no kqueue in the way: ordering, FIFO among ties, insert/remove from every position, a randomized cross-check against `std::stable_sort` on the same key, and 20000 interleaved inserts and removes against a queue kept deep enough to reach the delete fixup's harder branches.
- Because in-order traversal is correct even when the tree is badly shaped, that test validates the **tree**, not just the order: parent links, search order, no red node with a red child, and equal black heights on every path, re-checked after every single operation. The checker was itself checked, by confirming it fails when the root is left red, when a new node is coloured black, and when a recolour is skipped.
- `//aos/events:aio_test` covers the contract across backends, including `OneCompletionPerPollTest` as the control that pins down the window the same-batch tests rely on.
- kqueue's stack ordering, its `EVFILT_READ`-nestability, and the non-inheritance of a kqueue fd across `fork(2)` were each established with a standalone probe before any code was written against them, rather than inferred from documentation.

## Lessons learned

- **A kernel's delivery order is not a contract until something asserts it.** Both Linux backends satisfied the ordering rule incidentally; nothing tested it until a platform that did not satisfy it turned up. `OneCompletionPerPollTest` exists so that the next backend finds out at test time.
- **Translating one readiness API into another is where the platform leaks.** Every macOS-specific defect in this work was in the translation layer — `EV_EOF`'s two meanings, unsubscribed readiness, `Aio`'s encoding mistaken for epoll's — and none was in the dispatch logic underneath. Keeping `events` equal to "the mask epoll would have produced" is what lets that dispatch stay a transcription.
- **Shared code pulled out from one platform breaks the others where that platform cannot see.** Factoring the common backend out of epoll and kqueue moved the readiness constants into `aos::internal`; `aio_linux.cc` was given the using-declarations for them and `aio_darwin.cc` was not, which is twenty compile errors that a Linux build reports as success. The same commit left `KqueueImpl::WaitForOne()` with no return on the timed-out path, with the check that should have caught it nested inside a branch where it cannot be true. Neither is a design mistake; both are what happens when the only compiler in the loop targets one of the platforms. Cross-compiling the other target is cheap and catches this class outright.
- **An order test cannot see a broken balance.** A red-black tree with a botched fixup still traverses in perfect order. Testing the structure's invariants, and then mutating the implementation to confirm the invariant checker actually fails, is the difference between a test and a decoration.
- **Not every mutation that survives is a coverage gap.** Deleting the insert fixup's zigzag straighten breaks no test, because the fixup loop re-enters, lands in the mirror case, and converges on a valid tree — more rotations, correct result. Worth tracing rather than assuming a hole.

## References

- ADR 0001: io_uring backend for Aio — the contract this backend is meeting.
- `aos/events/aio.h` — `OnEvents()`/`SetEvents()` event encoding, `Poll()`'s one-completion rule.
- `aos/events/aio_readiness_backend.h` — the shared readiness backend and the seam this one implements.
- `aos/events/timer_queue.h` — the ordering structure and its costs.
- `aos/events/aio_darwin.cc` — `UpdateTimerKnote()`, `DesiredReadiness()`, `ToMachTicks()`, and `Poll()`'s translation.
- `aos/events/aio_uv.h` — the borrowed-loop `fork(2)` limitation.
- `kqueue(2)`, `EVFILT_TIMER` `NOTE_MACHTIME`/`NOTE_ABSOLUTE`, and the note that a kqueue descriptor is not inherited by a child.
