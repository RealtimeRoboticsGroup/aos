# ADR 0001: io_uring backend for Aio — SINGLE_ISSUER/DEFER_TASKRUN with automatic per-instance downgrade

## Status

Proposed. Partially implemented.

This document describes the full design; not all of it exists yet. As of the change introducing it (`aos/events/aio.h`, `aos/events/aio_linux.cc`):

- **Implemented:** the io_uring backend itself, unconditionally — `SINGLE_ISSUER | DEFER_TASKRUN` with `R_DISABLED` lazy binding, the automatic per-instance downgrade (`DowngradeFromSingleIssuer()`), timerfd-backed timers, non-blocking orphan/recycle destruction, the embedded-epoll legacy-fd path, thread-signal receivers, and the enforced kernel version floor.
- **Not yet implemented** (referenced below; lands in follow-up changes): `fork()` support (decision 6 and its design section — `pthread_atfork`, `CheckForFork()`, `HandleFork()`, `CheckForParentFork()`); the `--aio_backend` flag and the `EpollImpl` epoll backend (`Aio::Aio()` today constructs `IoUringImpl` unconditionally, `aio_test.cc`'s `AioBackends` parameterization currently has the single value `true`, and every "both backends" statement below describes the contract the follow-up must meet); moving `ShmEventLoop` and `EPoll` onto `Aio` (both still run directly on `aos/events/epoll.h` today); and the `WPILibRobotBase::AddLoop()` factory restructure under `frc/`.
- **Superseded detail:** the _enforced_ kernel floor is the 6.1 feature floor (`RequireMinimumKernelVersion()` in `aos/events/aio_linux.cc`), with 6.12 recommended in the error message; the "Kernel version floor: 6.12" section below predates that requirement/recommendation split.

## Context

`Aio` (`aos/events/aio.h`) is AOS's cross-platform completion-based async I/O primitive. `ShmEventLoop` and `EPoll` are intended to be built on top of it (see Status for where that stands). On Linux it has two backends, selected at runtime by `--aio_backend` (`IoUringImpl`/`EpollImpl` in `aos/events/aio_linux.cc`): an `io_uring` backend (the default) and an `epoll` backend. The flag is authoritative; there is no automatic fallback between backends, and no availability probe — `io_uring` means io_uring or a loud constructor crash naming the kernel floor. Its default is compiled in from the target platform's `//tools/platforms/io_uring` constraint, so a build for a target that cannot run io_uring defaults to `epoll` without anyone having to remember a flag.

The interesting choice is the ring configuration. The unconstrained configuration (`IORING_SETUP_COOP_TASKRUN | IORING_SETUP_TASKRUN_FLAG`) imposes no threading requirements, but completion delivery under it is less predictable than under `SINGLE_ISSUER | DEFER_TASKRUN`: the kernel runs outstanding completion work "at the end of any system call or thread interrupt", which is fine for throughput but not the most deterministic timing behavior available.

`IORING_SETUP_SINGLE_ISSUER` combined with `IORING_SETUP_DEFER_TASKRUN` is the more deterministic mode `io_uring` offers. The kernel only processes outstanding work when the application explicitly asks for it, via a GETEVENTS-flagged call, rather than opportunistically. The catch: `SINGLE_ISSUER` permanently binds one specific OS thread as the ring's only allowed submitter, for the ring's entire lifetime.

Most `Aio` owners have one thread that covers the whole lifetime and can take that binding for free. Some don't. A few test helpers construct an event loop on one thread, hand it to a worker thread to `Run()`, and tear it back down on the original thread after the worker exits. For those, `SINGLE_ISSUER` can't be bound at construction time — nothing knows in advance which thread will end up calling `Run()`.

This ADR documents the design that gets `SINGLE_ISSUER`'s determinism by default without forcing every such caller to restructure.

### What the comparisons in this document are relative to

This document compares the design against four different baselines, and it matters which one a given sentence means:

- **`EPoll` (`aos/events/epoll.h`)** — the readiness-based loop AOS ran on before `Aio` existed, with `timerfd`-backed `TimerFd` timers, and what `ShmEventLoop` still runs on as of the change introducing this document. The "legacy fd" API (`OnReadable`/`OnWritable`/…) is its API. When a sentence says something "has always" been the case, it is `EPoll`'s behavior it means.
- **`EpollImpl`** — the planned epoll _backend of `Aio`_, not yet implemented. "Both backends", "the other backend", and "the same observable behavior on every backend" refer to this: the contract the follow-up must satisfy, verified today only against the io_uring side.
- **The unconstrained ring configuration**, `IORING_SETUP_COOP_TASKRUN | IORING_SETUP_TASKRUN_FLAG` — the configuration the automatic per-instance downgrade falls back to. "More deterministic", "less predictable", and "loses `SINGLE_ISSUER`'s benefit" compare `SINGLE_ISSUER | DEFER_TASKRUN` against this.
- **Earlier, unmerged iterations of this same io_uring backend** — the design went through several shapes during development (timers on `IORING_OP_TIMEOUT`, an availability-probe ring per construction, cancel-and-resubmit for legacy-fd mask changes, a draining `ReapCompletions()`) that never reached `main`. "The earlier design", "the old …", "the … this replaces", "pre-change", and the whole of "Lessons learned" refer to these. Nothing in the repository corresponds to them; they are recorded here so the reasons they were abandoned survive.

Where the baseline is not obvious from context, the text below names it.

## Decision

Use `IORING_SETUP_SINGLE_ISSUER | IORING_SETUP_DEFER_TASKRUN` as the primary ring configuration, on kernel >= 6.12, with an automatic per-instance fallback for the threading shape it can't support:

1. The ring is created disabled (`IORING_SETUP_R_DISABLED`) so construction itself doesn't bind a thread. Binding happens lazily, the first time the ring is actually driven.
2. Anything that owns an `Aio` bound to `SINGLE_ISSUER` must be destroyed on the same thread that first drove it. Enforced with an `ABSL_CHECK`, not left as an unenforced convention.
3. If an instance's construction thread differs from the thread that first drives it, that one instance transparently downgrades to an unconstrained `COOP_TASKRUN` ring instead of binding at all.
4. Timers are `timerfd`s that `io_uring` watches, not `io_uring` timeout ops. `Schedule()`/`Cancel()` are a single `timerfd_settime(2)` each and submit nothing to the ring. `Aio::Timer` is one-shot; callers that want a period re-arm from their own callback.
5. Nothing in the backend ever blocks on the kernel. Destruction orphans in-flight state instead of waiting for it.
6. `fork()` is supported, with the goal that the child gets a fully functioning `Aio` that shares nothing with the parent's: the child rebuilds its ring, its embedded epoll instance, and its timerfds; the parent resyncs `DEFER_TASKRUN` state.

## Design

### Kernel version floor: 6.12

`IORING_SETUP_SINGLE_ISSUER` requires kernel >= 6.0 and `IORING_SETUP_DEFER_TASKRUN` >= 6.1. The floor is deliberately set higher, at 6.12: that is the release where `PREEMPT_RT` was merged into mainline as `CONFIG_PREEMPT_RT`, no longer a separate out-of-tree patchset. A realtime robotics codebase has no reason to run on a kernel that predates that merge, so tying the `io_uring` floor to it adds no new practical constraint — it just makes an existing one explicit, and removes an entire fallback tier's worth of code paths.

There is deliberately no fallback within the configuration: if the flags fail on a kernel that claims to be new enough, that fails loudly (`ABSL_PCHECK`). The `COOP_TASKRUN | TASKRUN_FLAG` tier (reached only via the automatic downgrade) inherits the floor by construction — it is only ever reached by rebuilding a ring that already had to be created on the `SINGLE_ISSUER | DEFER_TASKRUN` tier.

The same reasoning is why runtime error completions on the persistent internal registrations (the wakeup read, the legacy-epoll poll, thread-signal receivers, and each timer's poll) are fatal rather than absorbed: an unsupported-feature `-EINVAL` surfaces as an immediate crash naming the op, never as a registration that silently stops firing.

### Ring binding: created disabled, bound on the first drive

`io_uring_setup(2)`, `io_uring_enable_rings(3)`, and liburing's own `test/single-issuer.c` together confirm that `SINGLE_ISSUER`'s thread binding is one-shot and permanent. There is no unbind or rebind API; the only way to change the bound thread is to tear the ring down and create a new one.

So the ring is created with `IORING_SETUP_R_DISABLED`. `EnsureBound()` runs at the top of every `Poll()` and, on the first call only, calls `io_uring_enable_rings()` — binding to whichever thread made that call. Running at the top of `Poll()` covers both `Run()`-based usage and the direct-`Poll()`-without-`Run()` pattern several tests use.

Submission before that first bind doesn't fail outright. `MaybeSubmit()` gates every non-essential `io_uring_submit()` on whether the ring is enabled. If it isn't, entries stay queued locally — `io_uring_get_sqe()` is pure userspace — and the first real submit after `EnsureBound()` flushes them. This is what lets pre-`Run()` scheduling work: `starter_test.cc`'s `SetupStarterCleanup()` arms a timer from the setup thread before a worker thread ever calls `Run()`.

`CheckSubmitterThread()` verifies the binding, called both from `~IoUringImpl()` directly and from `CheckForFork()` (which runs at the top of every public entry point that touches the ring). The explicit destructor call matters: without it a same-thread violation would only be caught indirectly, and then only as a cryptic kernel `-EEXIST` instead of a message naming the actual constraint.

### Same-thread destruction, and the PI-futex conflict

Requiring same-thread destruction conflicts with a separate, pre-existing AOS constraint.

AOS's shared-memory queues (`aos/ipc_lib/lockless_queue.cc`) use a `RobustOwnershipTracker`, backed by a Linux robust/PI futex, to track which sender/watcher/pinner slot an object owns. The futex is acquired in the constructor and released in the destructor. PI futexes are **kernel-enforced**: only the exact thread that locked one can unlock it. So every sender, watcher, and pinner must be destroyed on the thread that constructed it. That constraint has nothing to do with `io_uring` and predates this work.

Senders and watchers are normally constructed on the same thread as the `ShmEventLoop` that owns them. But `message_bridge_test_lib.cc`'s `ThreadedEventLoopRunner`/`PiNode` and `starter_test.cc`'s `ThreadedStarterRunner` construct the loop on one thread and run it on another. For those, `SINGLE_ISSUER`'s "destroy on `Run()`'s thread" and the PI futex's "destroy on the construction thread" point at different threads.

The resolution is automatic per-instance detection:

- `IoUringImpl` tracks `construction_tid_`, set once in the constructor (or in `HandleFork()`'s reconstruction).
- `EnsureBound()` compares the calling thread against it. If they differ, `DowngradeFromSingleIssuer()` tears the never-yet-enabled ring down, rebuilds on `COOP_TASKRUN | TASKRUN_FLAG` — no binding, no destructor-thread requirement — and re-arms everything registered so far (`ReArmPersistentRegistrations()`, shared with `HandleFork()`).
- Once downgraded, `CheckSubmitterThread()` is a no-op for that instance.
- The rebuild regenerates every _persistent_ registration from userspace bookkeeping. A raw `AsyncRead`/`AsyncWrite` submitted before the first drive has no registry to be re-armed from, so the downgrade counts them (`raw_requests_in_flight_`) and `CHECK`-fails with an actionable message rather than dropping one silently.

`WPILibRobotBase::RunLoops()` was the one _production_ shape affected, and the place `SINGLE_ISSUER` matters most. It has been restructured instead: `AddLoop()` takes a factory invoked on the per-loop thread, so construction, `Run()`, and destruction share a thread and every loop keeps `SINGLE_ISSUER`. The test helpers stay on the downgrade deliberately.

### Timers: a `timerfd` that `io_uring` watches

Each `Aio::Timer` owns a `timerfd`. `io_uring`'s only job is telling us when it becomes readable, via one multishot poll armed once in `Initialize()` and held for the state's whole life.

- `Schedule(deadline)` is a single `timerfd_settime(2)` with `TFD_TIMER_ABSTIME` and a zero `it_interval`.
- `Cancel()` is a single `timerfd_settime(2)` with a zero `it_value`.
- Neither submits anything to the ring. Both are RT-safe: one syscall, no allocation, no asynchronous tail to reconcile.

Multishot poll is correct here for the same reason it is correct for `ThreadSignalReceiverState`'s signalfd and _incorrect_ for `legacy_epoll_fd_`: a timerfd's readiness genuinely toggles on every read. `read()` zeroes `ctx->ticks` (`fs/timerfd.c`), and the next expiration calls `wake_up_locked_poll()` — a real wait-queue edge, which is the only thing `IORING_OP_POLL_ADD` ever completes on. So the steady state costs **zero submissions per firing**.

Three properties follow, and they are the reasons for this design:

**The deadline is honored exactly.** An absolute `CLOCK_MONOTONIC` deadline is handed straight to the kernel. A caller-driven periodic timer — re-arming from its own callback against a grid it owns, which is what `ShmTimerHandler` does — therefore never accumulates phase error, no matter how long it runs or how late any individual firing is. `RepeatingTimerHoldsPhaseTest` holds both backends to this across 500 firings.

**Rescheduling cannot lose a race.** `timerfd_settime(2)` atomically replaces whatever the kernel had, with no lookup to miss. It also zeroes the expiration counter, so a previous schedule's firing cannot leak into the new one. There is no cancel to race, no update to lose, and no ack to reconcile.

**A late-cancelled firing is suppressed for free.** The queued completion only says "the fd was readable"; the authority on whether a firing is still owed is the `read(2)` at dispatch time. A timer canceled or rescheduled after its completion was queued reads `EAGAIN` and delivers nothing, with no bookkeeping to invalidate. See "Dispatch" below for why that window exists.

The cost is one fd per timer and one `read(2)` per firing — the price `EPoll`'s `TimerFd` has always paid, and what the planned `EpollImpl` will pay too, so both `Aio` backends share a mechanism rather than approximating each other. Constructing a timer touches the ring (it arms the poll), so an app needs `--aio_queue_depth` at least as large as its concurrent timer count; it fails loudly at construction rather than subtly later.

`Aio::Timer` is deliberately one-shot. A periodic timer that merely re-arms itself is not enough on its own: the owner still has to decide what happens to periods that elapsed while it was busy, and that policy genuinely differs (`ShmTimerHandler` skips to the next future deadline; `PhasedLoop` counts the missed ones). Implementing the repeat inside `Aio` means owning that policy for everyone and being wrong for someone. See "Lessons" for what that cost when it was tried.

What this does **not** fix: delivery is still gated on the loop reaching `io_uring_enter()`. A timerfd firing wakes a blocked `DEFER_TASKRUN` waiter promptly (`io_req_local_work_add()` is called without `IOU_F_TWQ_LAZY_WAKE`, so it always `wake_up_state()`s the submitter), but a loop busy in a callback won't see it until it comes back. That is inherent to a single-threaded event loop, and it is a per-firing delay rather than a permanent phase shift — the grid does not move.

### Dispatch: two phases, and the window that creates

Completion callbacks routinely reenter the ring: a timer callback reschedules, a callback can even `Poll()`. Dispatching straight off the CQ iteration would let that reentry advance the CQ head under the live scan, or recurse into nested dispatch. So handling is two-phase:

- `DrainCompletions()` extracts every available CQE in one scan plus a single bulk `io_uring_cq_advance()`, sets each request's `done`, and queues callbacks on `pending_dispatch_` — running no user code at all. That is what makes it safe to call at any nesting depth.
- `ReapCompletions()` dispatches exactly one queued callback and returns, leaving the rest for the next `Poll()`. `dispatching_` turns a nested attempt into queue-for-the-outer-loop instead of recursion.

The window this creates is real and must be handled: between "a completion is queued" and "its callback ran", an _earlier_ callback in the same batch can cancel, reschedule, or destroy that timer. All three are ordinary patterns. Cancel and reschedule are answered structurally by the read-at-dispatch-time rule above. Destruction is different — it frees the state the dispatch loop would walk into — so `DestroyTimerState()` unlinks from `pending_dispatch_` unconditionally, matching what `CancelRequest()` has always done. `DestroyTimerWithTerminatedPollAndQueuedCompletion` reproduces the use-after-free without that unlink, as a SIGSEGV inside `OnTimerFdReadable()`.

Dispatching one at a time is what keeps that window identical on every backend, which is why it is a documented contract on `aos::Aio::Poll()` rather than a property of this backend. `EpollImpl::Poll()` gets there by asking the kernel for one event at a time (`epoll_wait(..., maxevents=1)`); this one gets there by draining the whole CQ — it has to, or the CQ overflows — and then dispatching a single entry off `pending_dispatch_`. Different mechanisms, same observable behavior: a consumer cannot tell the backends apart by counting callbacks, and the same-batch tests below mean the same thing on both.

An earlier, unmerged iteration of this backend let `ReapCompletions()` drain its queue, so io_uring delivered two due timers from one `Poll()` and epoll delivered one. That difference was real, consumer-visible, and asserted by a parameterized test that branched on the backend — which is the shape of a leak in the abstraction, not a fact about it. It also made the three same-batch tests silently vacuous on epoll, where the window they name never opened. `OneCompletionPerPollTest` is the control that keeps them honest, and it now asserts the same thing for both.

Draining N ready completions therefore takes N `Poll()` calls. The cost is a loop iteration each, not a syscall each: `Poll()` returns early on a non-empty `pending_dispatch_` without entering the kernel at all (it still flushes staged SQEs, so work a callback submitted is not stuck behind the queue), and no new CQE can arrive while it isn't entering, so the queue strictly shrinks. `Run()` is an unconditional drain loop on both backends, so nothing falls behind.

### Destruction: nothing blocks, callbacks never fire

- Destroying a `Timer` (or unregistering a thread-signal receiver) with kernel traffic still in flight strips the user callback, makes sure a cancel is in flight, and parks the state on an intrusive orphan list. Once its terminal completion has drained (and dispatched or been unlinked), the state is recycled onto a freelist (`DestroyTimerState()` / `RecycleDrainedOrphans()`). Cancel acks never gate recycling — they carry `cancel_ack_sentinel_`'s identity, not the orphan's (see checklist item 2). Recycling is pointer manipulation only, so the sweep runs inside `Poll()` on RT threads; memory is bounded by peak usage and actually freed only in `~IoUringImpl()`. A recycled timer keeps its already-created timerfd.
- `~IoUringImpl()` submits no per-request cancels at all. `io_uring_queue_exit()` reaps every in-flight op kernel-side, and nothing ever drains the ring again, so no CQE can be observed after that point.

The visible contract this implies (documented on `aos::Aio`): completion callbacks only ever run inside `Poll()`/`Run()`. Destroying the `Aio` terminates pending requests kernel-side but does not finalize them — no callback, no Canceled completion. A caller that needs to observe a raw `AsyncRead`/`AsyncWrite` finish must `Cancel()` it and keep `Poll()`ing until the callback runs, before destruction. `Timer::Cancel()` is likewise silent by design.

Destruction is _not_ RT-safe, and says so: `aos::CheckNotRealtime()` guards `DestroyTimerState()` and `UnregisterThreadSignalReceiver()`. Not because anything blocks — nothing does — but because destruction can free, and a path that only _sometimes_ frees would only sometimes trip the RT malloc hook, which is a data-dependent crash. Making it deterministically illegal is better than making it usually fine.

### Legacy fd registration: one embedded epoll instance

`OnReadable`/`OnWritable`/`OnError`/`OnEvents`/`EnableWritable`/`DisableWritable`/`SetEvents` — the readiness-based API `EPoll` exposes, kept so `EPoll` and its consumers can move onto `Aio` unchanged — are backed by a single embedded `epoll_create1()` instance (`legacy_epoll_fd_`). Registration and mask changes are plain `epoll_ctl()` calls: no `io_uring` interaction, no cancel-and-resubmit, since the kernel supports updating an armed epoll registration in place. `io_uring` is used for exactly one thing — knowing when `legacy_epoll_fd_` itself has something ready — which is one registration total regardless of how many fds are registered.

That poll is **single-shot, explicitly re-armed on every firing** (`SubmitLegacyEpollPoll()`), unlike every other multishot registration in the file. `IORING_OP_POLL_ADD` only completes on a fresh wait-queue edge, never merely because a condition is still true, and `legacy_epoll_fd_`'s readiness does not toggle on read the way a signalfd's or timerfd's does. Arming a _fresh_ poll re-checks current state at arm time — the same "already ready" fast path `poll(2)` has — so it fires immediately if the condition never went away. See "Lessons" for the two live hangs that established this.

`DrainLegacyEpoll()` processes one event per firing. Fairness comes from level-triggered epoll itself: a still-ready fd goes back on the ready list's tail, so successive firings rotate through every ready fd, and legacy fds stay fair against the ring's native work. It is deliberately not a drain-to-empty loop, which busy-spins forever on any fd whose callback doesn't consume its own readiness.

### `fork()`: rebuild in the child, resync in the parent

Saying no to `fork()` isn't an option: `starterd` forks every application it manages while its own event loop is live, and death tests fork the test process. But io_urings don't work across forks — the SQ/CQ mmaps are shared with the parent, not copied — so a child using the inherited ring races the parent for the same queues.

**What the child is entitled to.** The goal is the strongest one: after a `fork()`, the child has a _fully functioning_ `Aio` that shares no kernel state with the parent's — it may drain events, keep running its loop, destroy things, or `exec`, and whichever it does must neither observe nor disturb the parent. The weaker goals are all covered by that one, and each of the in-tree forkers needs a different one of them:

- `starterd` needs "immediate `exec`". That requires nothing from `HandleFork()` at all: every fd the backend owns is `CLOEXEC` (the ring fd unconditionally — `io_uring_setup(2)` allocates it with `O_CLOEXEC` — `legacy_epoll_fd_` via `EPOLL_CLOEXEC`, timerfds via `TFD_CLOEXEC`), and `Subprocess` does not touch the event loop between `fork()` and `execvp()`.
- Death tests need "some limited processing on the inherited `Aio`, then die" — the child registers fds, constructs and destroys timers, or `Poll()`s the parent's `Aio` before the `CHECK` it exists to exercise fires. That is the case that must not disturb the parent, because the parent is the test that goes on to report a result.
- "Destructors run in the child" is a subset of the above and is what `EXPECT_DEATH(loop.reset(), ...)`-style tests do.
- Nothing in tree today needs a child that keeps a long-lived event loop running on the inherited `Aio`, but that is the only goal under which the child's correctness does not depend on _what_ it happens to do, so it is the one the design targets. It is also what `EPoll`'s Darwin implementation already does (`AtForkHandler` in `epoll_darwin.cc` rebuilds every `TimerFd` and `EPoll` in the child).

Two `pthread_atfork` handlers bump global counters, one on each side. `CheckForFork()` runs at the top of every public entry point, destructors included.

**Child side.** `HandleFork()` rebuilds everything the kernel shares between the two processes. All three kinds of inherited object have the same problem, so all three are replaced rather than reused:

- The ring. Its SQ/CQ mmaps are `MAP_SHARED`, so a child submitting to it writes into the parent's queues, and under `SINGLE_ISSUER` the child's first `io_uring_enter()` would either fail with `-EEXIST` or — on a not-yet-enabled ring — bind the child as the submitter and poison the ring for the parent. The child creates a fresh ring; closing the inherited fd only drops a reference.
- `legacy_epoll_fd_`. An epoll instance is one kernel object with one ready list and one interest set: both processes' `epoll_wait()`s would consume from the same ready list, and an `epoll_ctl()` in either would change what the other is watching. The child creates a fresh instance and re-registers its fds.
- Each timer's timerfd. Exactly the same shape, and the reason it is easy to get wrong is that the failure is quiet: a timerfd is one kernel object with one expiration counter and one armed deadline. A `read(2)` in either process zeroes the counter, so whichever process reads first takes the firing and the other reads `EAGAIN` — which this design deliberately treats as "not owed, deliver nothing" (see "Dispatch"). A `Schedule()`/`Cancel()` in either process retargets the other's timer, so a child that merely _destroys_ a timer on the way out (which disarms the fd) would silently cancel the parent's. The first version of this section claimed timerfds "need nothing" because they survive the fork still armed; that is true and beside the point — surviving is the problem, not the solution. The child creates a fresh timerfd per timer, re-arms it with the deadline the timer had _at the fork_, and re-submits the poll on the new fd. That deadline has to come from userspace bookkeeping — `Schedule()`/`Cancel()` record the armed absolute deadline (or "disarmed") on the timer state alongside the `timerfd_settime(2)` call — not from `timerfd_gettime(2)` on the inherited fd. The inherited fd is still the parent's live timer: by the time the child's first entry point runs `HandleFork()`, the parent may already have rescheduled or cancelled it, so reading it back would copy the parent's _current_ state rather than the forked one. The copy-on-write memory image is the only snapshot that is exactly as of the fork, which is the same reason every other persistent registration is regenerated from userspace bookkeeping too.

Then every persistent registration is re-armed on the new ring (`ReArmPersistentRegistrations()`, shared with the downgrade path). `CheckForFork()` must run before `EnsureBound()` in `Poll()` — otherwise whichever process polls first would consume the still-shared ring's one-shot enable and poison it for the other — and before `CheckSubmitterThread()` in the destructor, since the child's tid is never the parent's bound tid.

**Parent side.** This one is a genuine kernel quirk. Under `DEFER_TASKRUN`, task work is only ever run from inside `io_uring_enter()` with `IORING_ENTER_GETEVENTS` — `io_req_local_work_add()` (`io_uring/tw.c`) deliberately does _not_ force the task to process it on return from an arbitrary syscall, which is the whole point of the mode. So anything that takes the process away from its `Poll()` loop for a while leaves task work parked on `ctx->work_llist`, and any kernel-side state that task work would have updated stays stale for exactly that long, with no bound.

`CheckForParentFork()` closes it: after any fork this process was the parent in, the next public entry point makes one unconditional `io_uring_submit_and_get_events()` call, which runs the backlogged local work. It uses `submit_and_get_events` rather than a bare `io_uring_get_events()` deliberately — the latter hardcodes `to_submit=0` and would never flush SQEs that `MaybeSubmit()` left staged before the ring was enabled. A bare `get_events` left a ~1% residual failure rate under stress; `submit_and_get_events` was clean across 7000+ iterations.

### Ring teardown is throughput-capped

Closing a ring's fd is fire-and-forget. `io_uring_release()` queues `io_ring_exit_work` on a dedicated workqueue (`alloc_workqueue("iou_exit", WQ_UNBOUND, 64)` — at most 64 concurrent teardowns) and returns. Until teardown completes, _everything_ the ring pinned stays allocated: the ctx, every cached `io_kiocb`, the SQ/CQ ring memory, and the ring's file/dentry.

**Every ring's teardown costs at least one RCU grace period, regardless of flags.** `io_ring_ctx_wait_and_kill()` calls `percpu_ref_kill(&ctx->refs)`, whose switch to atomic mode goes through `call_rcu` (`lib/percpu-refcount.c`). The refs cannot drain, and nothing can be freed, until that grace period passes. `DEFER_TASKRUN` rings then pay a _second_, explicit `synchronize_rcu()` at the end of `io_ring_exit_work`. So disabling `SINGLE_ISSUER`/`DEFER_TASKRUN` roughly doubles teardown throughput; it does not remove the cap — a full stress run on `COOP_TASKRUN` showed 17M live `io_kiocb` and 32 GB of `SUnreclaim` mid-run, worse in objects than the `DEFER_TASKRUN`-era peak, because rings from both configurations retire through the same gated pipeline.

The cap is on the order of 64 rings per RCU grace period: thousands per second on an idle machine (~25 ms grace periods), collapsing toward double digits on a CPU-saturated one where grace periods stretch to a second or more. Ring _creation_ has no corresponding limit, and the failure mode is a feedback loop — the resulting memory pressure slows grace periods further.

This was found from a `--runs_per_test=10000` stress run: ~25 concurrent test actions per build-cluster node, each binary creating 4,213 rings, driving ~13,000 ring creations/s per node. One node accumulated a ~2M-ring backlog — 36 GB of `SUnreclaim`, 10.4M live `io_kiocb`, and active OOM-kills of unrelated Kubernetes pods. When the load stopped it drained at almost exactly the predicted idle cap and the machine recovered without a reboot, confirming deferred-but-capped cleanup rather than a leak.

There is no API to wait for a specific ring's teardown; after `close()` the object is deliberately unobservable. The only real lever is creating fewer rings. Two mitigations are in place: `Aio::Aio()` creates exactly one ring (an earlier, unmerged iteration probed io_uring availability by creating and destroying a throwaway ring per construction), and ring-churning test loops self-throttle via `ThrottleOnKernelRingTeardown()`, which watches `/proc/meminfo`'s `SUnreclaim` and sleeps when the backlog grows. That is backpressure only, never an assertion: the counter is machine-global, so neighboring processes can only cause extra throttling, not a wrong pass.

Production robots are essentially unaffected — long-lived processes create a handful of rings at startup and hold them. This is specifically a hazard for massively parallel CI, and for any future design that creates rings per-connection or per-request. The clean kernel-side fix would be freeing the ctx via `call_rcu` rather than blocking `io_ring_exit_work` on `synchronize_rcu`; worth raising upstream.

## Standing checklist: where this design fights the kernel

Places where the design's assumptions and the kernel's actual contracts pull in opposite directions. New code in this file should be checked against this list.

**1. Identifying in-flight ops by pointer, then retargeting them.** `IORING_OP_ASYNC_CANCEL` looks its target up by identity at submit time, and an op that is mid-transition can be briefly invisible to that lookup. Reusing one identity across successive incarnations is only safe if a cancel aimed at incarnation N can never land on N+1.

The strongest answer turned out to be structural: **don't retarget in-flight ops.** Timers were the only thing that ever needed to, and they are timerfds now, retargeted by a syscall with no lookup at all. What remains cancellable — polls and raw reads/writes — is only ever cancelled at teardown.

Generation-tagged `user_data` guards the residue: every `user_data` encodes `[63:48] generation | [47:2] pointer | [1:0] ack tag`, with the generation bumped on each fresh submission. The kernel matches a cancel's target by _exact_ `user_data` compare (`io_cancel_req_match()`), so a stale cancel structurally cannot match a successor — no userspace discipline required. Every target CQE self-identifies its incarnation, and any trace names `(pointer, generation)`. With acks carrying the sentinel identity (item 2), nothing can be legitimately stale anymore — arms are sequenced behind drained terminals — so a generation mismatch is a fatal invariant violation, not a drop path. Wrap-around cannot re-create ambiguity: it would take 65536 incarnations of one identity queued between two `io_uring_enter` calls.

Identities must stay alive until their terminal CQE has drained — which is the same event that makes their callback eligible to dispatch, so the public rule collapses to "valid until the callback runs" (`aos::Aio` constraint 2). The terminal is the _last_ CQE that names an identity, because acks deliberately do not (item 2); the orphan lists cover the terminal without blocking.

**2. Discarding kernel acks.** An ack is the kernel telling you `-ENOENT` — "your target wasn't there; what you meant to happen, didn't." Cancel acks are ack-tagged and validated (`0`, `-EALREADY`, or `-ENOENT`; anything else dies loudly). A null or unrecognized `user_data` is a hard `CHECK` failure, not a silently skipped entry.

Acks deliberately carry a loop-owned sentinel identity (`cancel_ack_sentinel_`), **not** their target's. An ack that named its target would require the target to outlive the ack's drain — and that drain is unobservable: a CQ overflow can push it past the target's own Canceled callback, arbitrarily many `Poll()`s later. Naming the target therefore either leaks an internal event into the public lifetime contract ("keep the request alive until... something you can't see") or forces ack-counting machinery to gate every orphan free. Naming the immortal sentinel removes the whole category: the terminal completion is the last CQE that names any identity, callers free after the callback, orphans recycle on their terminal alone.

The history matters here, because the target-tagging this replaces was not arbitrary. In the abandoned retargeting design (see Lessons), acks were _actionable, per-target signals_: reschedules and cancels resolved by lookup, a miss had to be reconciled against the specific op that missed, and an early fix that retried cancels off the ack killed successor incarnations — found live, not hypothesized — precisely because identities are reused. Target+generation tagging was the fix that made ack validation and stale-ack drops exact. Once timers moved to timerfds, nothing _acts_ on an ack anymore: the only cancels left are teardown cancels of ops nothing will re-arm, a missed one has nothing left to kill, and the tagging survived as validation plus the orphan gate it itself necessitated. The sentinel is only safe because of that: **if a future change reintroduces cancel-and-reconcile — any code path that must act on a specific cancel's outcome — it must also reintroduce target-identified acks, and with them the identity-must-outlive-its-ack lifetime problem this scheme exists to avoid.** The sentinel makes an ack-driven retry structurally unwritable (the handler cannot name a target), which is a stronger guard against the successor-kill bug than the discipline it replaces; the one real cost is attribution, since an anomalous ack result now dies with a message that cannot name the request it answered.

A cancel ack is still never retried, and never ignored: results are validated, and an impl-level outstanding-ack count backs a `CHECK` that acks never arrive unexpectedly (reset on ring rebuild, where a dead ring's acks can never arrive).

**3. Treating multishot ops as persistent.** The kernel terminates _any_ multishot op whose per-firing CQE cannot be posted: `io_req_post_cqe()` returns false on a full CQ, and those auxiliary CQEs get no overflow-list fallback (unlike ordinary completions, which are preserved). "Persistent" ops are revocable at the kernel's convenience, so **every multishot consumer must re-arm on an unexpected terminal completion.** There are two: the thread-signal receiver's poll, and each timer's poll on its timerfd. Both re-arm, with deterministic regression tests forcing the overflow.

Re-arming applies to _successful_ terminals only. An **error** completion on a persistent internal registration is fatal instead: re-arming would resubmit the same failing op in a silent infinite loop, and absorbing it would silently kill the registration — the exact failure this item exists to prevent. The single-shot-rearm consumers (wakeup read, legacy-epoll poll) are structurally immune, since their CQEs are ordinary completions the overflow list preserves.

**4. Acting on a completion that exists but hasn't been delivered.** See "Dispatch" above. Two-phase dispatch creates a window between queueing and delivery in which an earlier callback can act on a later one's timer. Answered structurally for cancel/reschedule (read-at-dispatch-time) and by unconditional unlinking for destruction.

**5. One syscall for submit and wait.** `io_uring_enter` reports submission success over wait failure (`ret2 = io_cqring_wait(...); if (!ret) ret = ret2;` with `ret` holding the submit count). A combined call's return says nothing about the wait: an interrupted wait after submitting N SQEs returns plain `N`, indistinguishable from success, with no error reaching userspace at all. `Poll(true)` therefore loops until a completion is actually present (`io_uring_cq_ready() > 0`), not merely until a wait call returns. After the first iteration flushes the SQ, subsequent calls submit nothing, so a bare `-EINTR` becomes visible and retryable. Any new blocking path must follow the same rule: verify the thing waited for exists; never trust the call to have blocked.

**6. Ring-per-instance lifecycle vs. throughput-capped teardown**, and **7. `fork()` vs. inheritance of the ring, the epoll instance, and every timerfd.** Covered in their own sections above. They are the same posture — assuming cheap, transparent lifecycle operations that the kernel actually meters or ignores — applied to construction and process management rather than I/O.

## Consequences

Positive:

- The default on platforms that support it (`--aio_backend=io_uring`, `SINGLE_ISSUER`/`DEFER_TASKRUN`) gives more deterministic completion delivery than either the unconstrained ring configuration or `EPoll`'s `epoll_wait()` for the common case: everything constructed and run on one thread, which describes all current production usage.
- The same-thread destructor requirement is an enforced, checked invariant with a clear error message, not an unwritten assumption.
- `--aio_backend=epoll` remains a fully unconstrained fallback if the io_uring backend's behavior is ever a problem for a new use case.
- Test helpers that construct on one thread and run on another keep working unmodified, via automatic downgrade.
- Timer deadlines are honored exactly on both backends, and a timer reschedule cannot lose a race with its own firing. Both are relative to the abandoned `IORING_OP_TIMEOUT`-based iteration of this backend (see "Lessons"), not to `EPoll`, whose `TimerFd` always had these properties. Scheduling and cancelling submit nothing to the ring, so the whole class of "the cancel/update lookup missed its target" bug is absent rather than compensated for — along with the code that compensated.
- Both backends run timers on the same mechanism, so they share semantics instead of approximating each other.
- Legacy fd mask changes are a plain `epoll_ctl()` call, as in `EPoll`, rather than the blocking cancel-and-resubmit an earlier iteration of this backend used — closing the RT-safety gap that iteration had opened (its `CancelRequest(reap=true)` could fall back to an undeadlined `io_uring_wait_cqe()`; never triggered in production).

Trade-offs:

- An `Aio` that hits the construction/`Run()`-thread mismatch downgrades to the unconstrained ring configuration silently — there is no log line on downgrade today. Fine for test helpers; worth surfacing if a _production_ path ever downgrades unexpectedly.
- The kernel floor (6.12) is hard, with no fallback. An older kernel fails loudly at startup rather than degrading.
- Ring teardown is throughput-capped and RCU-gated regardless of configuration, and `DEFER_TASKRUN` roughly doubles the cost. Irrelevant for long-lived production processes; a real hazard for anything that churns rings at scale.
- Each timer costs a file descriptor and one `read(2)` per firing. It does _not_ cost a submission per period. Constructing a timer touches the ring, so `--aio_queue_depth` must be at least the concurrent timer count.
- `Aio::Timer::Schedule()` has no repeating form. A caller that wants a period writes the three-line re-arm itself (see `RepeatingTimer` in `aio_test.cc`, or `ShmTimerHandler`). `TimerHandler::Schedule(base, repeat_offset)` — the API robot code actually uses — is unchanged.
- `WPILibRobotBase::AddLoop()` is a breaking API change: it takes a factory instead of an already-built `ShmEventLoop*`. There are zero in-tree callers of the current (pre-factory) signature; out-of-tree robot code gets a compile error pointing at the new contract rather than a silent downgrade.

## Alternatives considered

**Restructure every affected call site so construction and `Run()` share a thread.** This remains the eventual end state; it removes the conflict at its source. The production-relevant piece is done (`WPILibRobotBase::RunLoops()`). The test helpers stay on the automatic downgrade deliberately: they gain nothing from determinism, and the downgrade path they exercise is itself regression coverage for a mechanism production code may still reach through future call patterns. Tracked by the `TODO(austin)` on `DowngradeFromSingleIssuer()`.

**A two-phase destruction handshake per call site.** The worker thread signals "`Run()` returned"; the owning thread destroys the PI-futex-owning objects, then signals the worker to destroy the bare `ShmEventLoop` and exit. Rejected in favor of the automatic downgrade: it would need independent implementation at each of three affected call sites, adding real synchronization complexity in multiple places, versus one shared mechanism in `IoUringImpl`.

**Keep `COOP_TASKRUN | TASKRUN_FLAG` as the only configuration.** Rejected: it gives up the determinism benefit entirely, for every caller, to avoid a conflict that has a much cheaper fix.

**Keep timers on `IORING_TIMEOUT_MULTISHOT` and correct the phase in userspace.** Considered once the drift was understood, since `ShmTimerHandler` already recomputes its deadline grid from the clock on every firing and was therefore masking the problem for `ShmEventLoop`. Rejected: it leaves `Aio::Timer`'s own contract wrong for every other caller, keeps all of the update/cancel lifecycle machinery that the fix otherwise deletes, and makes correctness depend on each consumer independently re-deriving what the kernel can get exactly right. The masking is also why the defect survived unexamined for so long — building on top of it would preserve that.

**Keep timers entirely in userspace and use `io_uring_enter(2)`'s own wait timeout as the wakeup** (`IORING_ENTER_EXT_ARG` with a `timespec`, and `IORING_ENTER_ABS_TIMER` so it can be the earliest absolute deadline directly). The loop would keep a min-heap of armed deadlines, block with the earliest one as the wait's timeout, and on every `Poll()` entry mark timers whose deadline has passed as due and dispatch them like completions. Rescheduling is trivial — update the heap and the "due" flag before blocking — and it costs no fd and no `read(2)` per timer at all. Not rejected; recorded here as the most credible future replacement, with the reasons it isn't the design today:

- _Kernel floor._ `IORING_ENTER_ABS_TIMER` is 6.12; the enforced floor is 6.1. Below 6.12 the timeout is relative only, which is workable (the relative value is recomputed from the clock right before each `enter`, so the error is one clock-read-to-hrtimer-start latency per _block_, not accumulated per _firing_ the way `IORING_TIMEOUT_MULTISHOT`'s was) but is a second code path to keep correct.
- _The heap is not free either._ Ordering armed deadlines and finding the earliest one has to happen somewhere. With timerfds it is the kernel's hrtimer `timerqueue`, and `Schedule()`/`Cancel()` stay a single syscall with no userspace data structure behind them. In userspace it is an intrusive min-heap on the RT path: allocation-free, but an `O(log n)` sift on every schedule and cancel, a scan or lazy-deletion scheme for cancels of non-root entries, and a recompute of the earliest deadline before every block. Small, but it is code that has to be RT-safe and correct rather than a kernel guarantee.
- _One mechanism across backends._ `EPoll` has always been timerfd-based, and the planned `EpollImpl` is; a shared mechanism is what lets the same timer tests mean the same thing on both. It was also chosen immediately after the `IORING_OP_TIMEOUT` drift was root-caused, when the fewest independently-verifiable moving parts won.

It would _not_ reopen the lifecycle races the timerfd closed. Those all came from retargeting a kernel-owned timeout op by identity through a lookup that can transiently miss (see "Retargeting in-flight ops by identity"). Here there is no kernel-owned timer object at all: the timeout is an argument to one `io_uring_enter` call and lives only for that call, and the timer state is userspace-only, mutated only by the loop thread. Reschedule and cancel are heap edits with nothing to look up; absolute deadlines come straight from the caller's grid, so there is nothing to drift; and the `fork()` rebuild has one fewer kind of object to replace. What it keeps is the _shape_ of two other rules in this document, which would need to hold in the new mechanism: never trust the wait's return (checklist item 5 — `-ETIME`, a completion, and an interrupted wait must all funnel into "is `now` past the earliest deadline?" against the heap), and decide due-ness at dispatch time against the heap rather than by a flag set when the wait returned, so a same-batch cancel delivers nothing (the userspace analogue of the read-at-dispatch rule). One thing to verify before adopting it: whether `io_cqring_wait()`'s hrtimer path applies `current->timer_slack_ns`. timerfd's `hrtimer_start()` does not; a `_range` sleep with default slack would give non-RT threads ~50 µs of wobble the current design doesn't have.

The per-timer cost this would remove — one fd, one `read(2)` per firing — is not something any current consumer notices. If it becomes one, this is the alternative to reach for.

**Poll the timerfd with a single-shot `IORING_OP_POLL_ADD` re-armed per firing**, as the legacy-fd poll does. Rejected in favor of multishot: a timerfd's readiness genuinely toggles on every `read()`, so it produces the real edge multishot needs. Multishot costs zero submissions per firing instead of one.

## Verification

- `--config=asan`/`ubsan`/`msan` builds.
- High `--runs_per_test` counts (typically 100–1000), both locally and via the remote build cluster. The cluster runs on the same hardware via Kubernetes, so it reproduces real contention a single local run doesn't.
- `git stash`-based before/after comparisons, to isolate whether a failure was pre-existing.
- `gdb`/core-dump root-causing on any crash, rather than guessing from a stack trace.
- Standalone raw-syscall reproductions when the question was about the kernel rather than this code. The multishot-timeout phase drift was established that way first — ~120 lines against `io_uring_setup`/`io_uring_enter` directly, comparing `MULTISHOT` against per-firing `IORING_TIMEOUT_ABS` over 200 periods — before anything in `aio_linux.cc` was touched.
- Confirming a new regression test actually fails against the code it regresses. `DestroyTimerWithTerminatedPollAndQueuedCompletion` reproduces as a SIGSEGV without its fix. `RepeatingTimerHoldsPhaseTest` reported 1.5 ms of accumulated phase error against the `IORING_OP_TIMEOUT`-based iteration while the repeating API still existed. Worth recording that the latter can no longer fail that way: removing the repeating API removed the construct that drifted, so the test now guards the property the design rests on rather than the specific defect.

The figures quoted in "Lessons" below (10,000-run gates, 25,020 remote runs) predate the timer redesign. That change has been re-verified against the full `//aos/...` suite and high `--runs_per_test` counts, not another 10,000-run gate.

See `aos/events/aio_test.cc` and `aos/events/shm_event_loop_test.cc` for the regression tests covering each mechanism.

## Lessons learned

### From designs that were abandoned

**The timer that could not keep time.** For most of this effort `Aio::Timer` was built directly on `IORING_OP_TIMEOUT`: a repeating timer's first firing handed off to a steady-state `IORING_TIMEOUT_MULTISHOT` op, and rescheduling an armed timer used `IORING_TIMEOUT_UPDATE` in place of cancel+rearm. It was the native, obviously-idiomatic way to express a timer in this API, and it was wrong in a way nobody was looking for.

`__io_timeout_prep()` (`linux/io_uring/timeout.c`) rejects `MULTISHOT | ABS` with `-EINVAL`, so a repeating timeout is _necessarily_ relative. `io_timeout_complete()` then re-arms it with `hrtimer_start(period)` from _now_ — not `hrtimer_forward()` from the previous deadline — and "now" is whenever the rearm's task work ran, which under `DEFER_TASKRUN` is when userspace next made a GETEVENTS call. Every period silently absorbs the hrtimer→wake→taskrun latency, and it accumulates without bound: measured at 4–10 µs per period on an idle machine, growing with how busy the loop is. A 10 ms timer slides ~1 ms per 200 periods, forever, with nothing that ever pulls it back.

The generalizable lesson: **the most deterministic-looking primitive is not automatically the one with the best timing.** The premise of this ADR is that `SINGLE_ISSUER | DEFER_TASKRUN` gives more predictable completion _delivery_, which is true and is why it stayed. It does not follow that building every timing primitive out of `io_uring` ops gets you predictable timing. A `timerfd` — older, less fashionable, one syscall and one `read()` per firing — is exactly correct on the property that actually mattered, because its rearm is `hrtimer_forward_now()` off the previous deadline. Identify the property you need and check the primitive against _that_, rather than assuming the more integrated mechanism inherits every desirable property of the subsystem it lives in.

Worth noticing how long this went unexamined: every timer bug in this file was a _lifecycle_ bug, and lifecycle bugs are loud. The quiet, monotonic, correctness-of-timing defect underneath them was never what anyone was looking at.

**The repeating-timer API nobody used.** `Aio::Timer`'s repeating mode had exactly one caller passing a non-zero period, and that caller (`ShmTimerHandler`) discarded it on the first firing and did its own deadline math from then on. It nonetheless dictated the shape of the timer code in three backends: the whole `IORING_TIMEOUT_MULTISHOT` state machine existed to serve it, along with a "deliver one callback per banked expiration" rule and the two guard members each backend needed because a single firing could then re-enter the user callback several times. Removing the parameter deleted all of it, and let each backend dispatch the user callback as the last thing it touches the state with — so "a callback destroys its own timer" needs no guard at all.

**A feature with no real consumer will still cost you correctness work.** When a primitive is hard to get right, ask whether anything is actually asking for the hard part. The answer here was visible from a five-minute grep, long before the drift was.

**Retargeting in-flight ops by identity.** `IORING_TIMEOUT_UPDATE` and `IORING_OP_ASYNC_CANCEL` both bottom out at `io_timeout_extract()`'s scan of `ctx->timeout_list` — a list that a firing op is briefly absent from, because `io_timeout_fn()` unlinks it in interrupt context and hands the rearm to task work. Under `DEFER_TASKRUN` nothing runs that task work until the next GETEVENTS call, so the window has no kernel-enforced bound. Essentially every lifecycle bug in this effort was a variant of losing that race, in three flavors:

- _Cancel vs. mid-rearm._ A synchronous reap loop submitted one `Cancel()` and then only drained. A cancel landing in the window was silently discarded and the target kept firing forever. Surfaced as a ~1-in-500 flake, then reproduced deterministically 1000/1000 by sleeping past the period with no `Poll()`.
- _Update vs. firing._ An `IORING_TIMEOUT_UPDATE` racing its target's firing missed, and the stale completion was dispatched carrying the _new_ schedule's callback but arriving _before_ the new deadline. `ShmEventLoop::HandleEvent()` correctly filtered it as a spurious early wakeup — and meanwhile the kernel op was gone. The timer died silently: armed in every userspace structure, absent from the kernel. ~1 in 10⁴ full-suite runs under node load, zero in isolation at any count.
- _Cancel vs. multishot firing after `fork()`._ Not a fork bug at all; a fork is just an easy way to make "time since the last GETEVENTS call" large.

Three separate compensating mechanisms were built for these before the structural fix. **The lesson is the shape, not the opcodes:** if a design needs to retarget something the kernel resolves by a lookup, and that lookup can transiently miss, every mechanism you build is a compensation rather than a fix. Ask whether the retargeting is necessary at all. Here it wasn't — `timerfd_settime(2)` replaces the whole thing atomically.

**A bounded retry loop that never retries the actual operation isn't a mitigation; it's a slower-motion version of the same hang.** The reap loop had a bound and a clear failure message — the exact shape this effort pushed for everywhere else. It still took 10,000 real, individually-legitimate completions to fail, because bounding _how long you wait_ is not the same as bounding _how many chances the actual fix gets to work_. The loop was draining real data the entire time, which is why the original comment's claim ("the only way this doesn't terminate is a genuine kernel problem") felt safe to believe. A bounded loop around an operation that can silently lose a race needs the retry _inside_ it, actually re-attempting the thing that can fail.

**The ordering proof.** A three-part argument — one kernel property (these opcodes execute inline in SQ order) plus two userspace invariants (arms are sequenced behind terminals; cancels are only submitted against a current victim) — proved that a stale cancel could never land on a successor incarnation. It was sound. It was also a property of _every arm and cancel site at once_, so any future edit could erode a premise without anyone noticing, and it described code that no longer exists. Generation-tagged `user_data` makes the same guarantee locally and unconditionally, needs no global argument, and is what production correctness actually rests on. **A global invariant proof is worth writing to check whether a design is coherent, and worth distrusting as the thing you rely on.**

**A blocking wait that didn't block.** `Poll(true)` used a single `io_uring_submit_and_wait()` and trusted its return. Found only after two full 10,000-run invocations produced 65 and then 84 failures — every one a timer test whose `while (cond && aio.Poll(true))` loop exited early with a stalled count, none reproducing in isolation (0 in 20,000 filtered runs), none carrying any error.

The mechanism took live instrumentation plus kernel source to assemble. Ring teardown delivers `io_tctx_exit_cb` via `task_work_add(..., TWA_SIGNAL)`, which interrupts whatever the task is doing — including the _current_ ring's blocking wait. A test suite's earlier rings tearing down therefore interrupt waits on the ring in use now, which is why fork-heavy sections were the casualties and isolation never reproduced anything. And `io_uring_enter()` discards the wait's return unless nothing was submitted, so the interruption reached userspace as a plain success. A first-attempt fix that retried `-EINTR` accomplished nothing (84 failures, statistically unchanged) — that failed fix was itself diagnostic, eliminating the visible-EINTR hypothesis and forcing the instrumentation that caught the real shape.

**The ring-creation retry, and the availability probe.** A transient `-ENOMEM` on ring creation under synchronized parallel startup got a narrowly scoped retry — `-ENOMEM` only, never a broad catch-all, after tracing the kernel's allocation path to confirm `-EAGAIN` isn't even reachable there. It was the right call at the time and the wrong thing to keep: the real cause was allocation running against gigabytes of unreclaimable teardown backlog, produced by an availability-probe ring created and destroyed on every `Aio` construction plus a 2000-iteration construction-race test. With creation cut roughly 10x, 10,000 full-suite runs with retries disabled produced not a single `-ENOMEM`, and the retry was deleted rather than left as unfalsifiable insurance. **A mitigation whose underlying cause has been fixed should be removed, not kept as insurance — otherwise it hides the cause's return.**

**Multishot poll on a level-triggered fd.** The legacy-fd redesign took two live hangs to get right, both the same lesson from opposite directions. First: multishot poll on `legacy_epoll_fd_` with a drain-to-empty loop hung immediately, because `LegacyFdTest`'s callback deliberately doesn't consume its pipe, so the fd never stopped being ready. Second: one batch per firing, no loop — hung later on `FillPipe()`, because a pipe with one more byte of room is still writable, nothing about writability _changed_, and so nothing woke the registration again.

**A wake-driven poll and a directly-called, repeatedly-polled one are not interchangeable, even when one is built on top of the other.** `IORING_OP_POLL_ADD` (multishot or not) only completes on an actual `wake_up()` from the polled file's wait queue. Direct, repeated `epoll_wait()`/`poll()`/`select()` calls have no such limitation: each call independently re-verifies current state, which is what makes classic level-triggered polling work regardless of wake history. Wrapping the first around the second silently inherits the edge-only limitation for the wrapping fd, even though everything downstream is still genuinely level-triggered. The fix wasn't a smarter loop; it was recognizing that the _outer_ poll needed re-arming discipline the _inner_ one never needed.

### General

**Two independently reasonable same-thread teardown rules, on objects that share an owner, aren't guaranteed to name the same thread.** `SINGLE_ISSUER` requires the `Aio` destroyed on the `Run()` thread; `RobustOwnershipTracker`'s PI futex requires senders/watchers destroyed on the construction thread. Neither rule is about the other's mechanism, and neither is unreasonable in isolation. The conflict exists only because `ShmEventLoop` owns both kinds of object, and it was invisible from either object's documentation or test suite — it surfaced only by restructuring code to satisfy one rule and hitting a live crash from the other. A new same-thread teardown requirement on one class can't be reasoned about by looking only at that class.

**The right response to a conflict like that isn't always "eliminate the conflicting pattern" everywhere.** The first instinct was to restructure every affected call site. That is invasive work across three subsystems for a benefit those specific call sites — test helpers — don't need. Automatic per-instance detection and graceful degradation was strictly better: production code gets the full benefit unconditionally, and test helpers get correct behavior without being rewritten.

**RT enforcement must be scoped to the exact call site that does the illegal thing**, not the general vicinity of one. An earlier draft of the `CheckNotRealtime()` guards also flagged ordinary reschedules, which broke `ShmTimerHandler`'s entirely legitimate RT-thread rearm pattern.

**`DEFER_TASKRUN`'s completion-delivery model is an easy-to-miss latency trap.** It defers processing until an `io_uring_enter(2)` call sets `IORING_ENTER_GETEVENTS`, which is _not_ automatic on every submit. Two real bugs came from this: a non-blocking `Poll()` path using bare `io_uring_submit()`, so an already-ready completion could silently never reach the CQ; and the parent-fork resync. Check every new non-blocking path against whether it needs `_and_get_events()`.

**`fork()` recovery not needing to be RT-fast doesn't mean it's exempt from correctness requirements.** "Not RT-fast" is not "eventually consistent for free": a fork's ordinary continuation can leave kernel-side bookkeeping _permanently_ wrong without an explicit resync. And "this fd survives the fork" is a hazard, not a convenience: any kernel object both processes can still reach — ring, epoll instance, timerfd — is one they will race on the moment either does more than `close()` it. The rule is that the child replaces every one of them, with no per-object argument for why this one is fine.

**Pinning a kernel version floor is a legitimate simplification for an RT robotics codebase**, not just a compatibility shortcut. Broad kernel-version compatibility costs real code paths to maintain and test. A codebase with a hard realtime dependency should spend that cost deliberately.

**Rare, load-dependent failures need a live reproduction, not a theory.** Every failure documented here — the `-ENOMEM` burst, the `DEFER_TASKRUN`+`fork()` quirk, the PI-futex crash, the reap-loop exhaustion, the phase drift, and a separate unrelated flake in `NextMessageNotAvailable` (root-caused to `PhasedLoop` dropping messages under scheduling delay, not an `Aio` issue at all) — was root-caused with direct instrumentation: temporary logging of actual errno values, minimal raw-syscall repros stripped of test-framework machinery, and cross-referencing kernel source. None were fixed by guessing. A retry loop or timeout is only trustworthy once the underlying failure mode is understood, and should then be scoped as narrowly as what was confirmed.

**Build the instrument the previous dead end demands.** The rarest bug here was found by a chain where each step's tool came from the last step's failure: an in-process hang watchdog (`aos/testing/hang_watchdog.h`, still load-bearing) that converted truncated remote logs into per-thread backtraces; `/proc/PID/fdinfo/<ring-fd>` for the kernel's own testimony about what was armed, which ruled out two whole theories at once; gdb member dumps showing every userspace structure healthy; attaching and detaching gdb to falsify a missed-wakeup theory by experiment, twice; and finally an env-gated event trace that caught the race in the act.

## References

- `io_uring_setup(2)`, `io_uring_enable_rings(3)` man pages (liburing).
- liburing `test/single-issuer.c` — confirms `SINGLE_ISSUER` binding is one-shot, and that the _enabling_ task becomes the bound submitter under `R_DISABLED`.
- Linux kernel source, `io_uring/timeout.c` (`__io_timeout_prep()` rejecting `MULTISHOT | ABS`; `io_timeout_complete()` re-arming with `hrtimer_start()` from task-work time rather than `hrtimer_forward()`) and `fs/timerfd.c` (`timerfd_setup()` zeroing `ctx->ticks`; the read path's `hrtimer_forward_now()`) — the phase-drift finding that put timers on timerfds.
- Linux kernel source, `io_uring/tw.c`/`tw.h` (`io_req_local_work_add()`, `io_req_normal_work_add()`), `io_uring/wait.c` (`io_cqring_wait()`), and `io_uring/io_uring.c`'s `io_uring_enter()` — the `DEFER_TASKRUN` deferral semantics behind the parent-fork resync and the blocking-wait fix.
- Linux kernel source, `io_uring/poll.c` (`io_poll_check_events()`) — multishot termination on a full CQ.
- Linux kernel source, `io_uring/io_uring.c`, `io_uring/memmap.c`, and `lib/percpu-refcount.c` — ring creation and the RCU-gated teardown cap.
- liburing `src/queue.c` (`io_uring_get_events()`, `io_uring_submit_and_get_events()`) — why the parent-fork resync needs the submitting variant.
- `aos/ipc_lib/lockless_queue.cc`'s `RobustOwnershipTracker` — the PI-futex mechanism this design coexists with.
