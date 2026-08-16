# Measuring Real-Time Performance

[How AOS is Real-Time](realtime.md) describes the mechanisms. This document is about verifying them on real hardware, and about what to do when a deadline gets missed.

The central problem with real-time debugging is that the interesting event happens once every few hours, lasts 300 µs, and is over long before you can attach anything to it. Everything below is organized around that: get a cheap always-on measurement first, then use it to _trigger_ an expensive capture at the moment of the violation — and [add load](#make-it-happen-more-often-load) so you are not waiting hours between chances.

## The ladder

Work down this list. Each rung is more expensive and more precise than the one above it.

| Rung | Tool | Cost | Answers |
| --- | --- | --- | --- |
| 0 | `cyclictest` | offline | Is the _kernel_ real-time at all? |
| 1 | Timing reports | always on, negligible | Which application/channel is late, and by how much? |
| 2 | `aos_jitter` | one extra process | Is a specific channel's timing degrading, and when? |
| 3 | **ftrace + trigger, read in KernelShark** | ~free until it fires | _Why_ was it late — what ran instead? |
| 4 | ping/pong | offline | Is the platform itself capable of the numbers you need? |

Rungs 0-2 are how you find out that something is wrong and roughly where. **Rung 3 is where real-time problems actually get solved.** Catch the violation in the act, freeze the trace, and look at what the machine was doing — that answers in one pass the questions that aggregated statistics can only narrow down. Expect to end up there for anything that isn't immediately obvious, and don't spend long theorizing at rungs 1 and 2 before going.

---

## Rung 0: Establish the floor

Before blaming AOS, find out what the kernel can do. Nothing in userspace fixes a kernel that can't schedule on time.

```bash
# Overnight, under load. -m locks memory, -p is the RT priority,
# -i/-d set the interval, -t one thread per core.
cyclictest -m -p 80 -i 200 -d 0 -t -a -h 400 --histfile=/tmp/cyclictest.hist
```

The max value is your floor. Every AOS latency number is that plus AOS's own contribution. If `cyclictest` shows 2 ms maxima, no amount of reading timing reports will help — go fix the kernel config, the driver, or the SMI/BIOS behavior first.

Confirm the kernel is actually `PREEMPT_RT`:

```bash
uname -a | grep -i 'PREEMPT_RT\|PREEMPT RT'
cat /sys/kernel/realtime 2>/dev/null   # "1" on an RT kernel
```

If `PREEMPT_RT` is not an option on your target, being real-time-shaped is still worth it — [Running without `PREEMPT_RT`](realtime.md#running-without-preempt_rt) covers what changes and which levers still work (mostly: `threadirqs` and pinning). The floor `cyclictest` reports will be higher, and everything below still applies against it.

---

## Rung 1: Timing reports

Every `EventLoop` maintains a timing report and publishes it on `/aos` once a second. This is on by default (`--timing_reports`, `--timing_report_ms=1000`) and is designed to have **no impact on RT performance**: the report flatbuffer is built once at startup and mutated in place, then `memcpy`'d into shared memory. No allocation, no formatting, no locks.

For each event, three times matter:

1. When the event _happened_ — the publish timestamp of a message, or the expiration time of a timer.
2. When the handler _started_.
3. When the handler _ended_.

From those come the two numbers you care about:

- **Wakeup latency** = (2) − (1). Scheduling delay, signal delivery, interrupt storms, priority inversion — the system keeping you from starting.
- **Handler time** = (3) − (2). Mostly your code, but **not only** your code. It is wall clock, not CPU time, so anything that preempted the handler is inside it: a higher-priority application, an IRQ thread, a softirq, a page fault. A handler that looks slow may just be one that keeps getting interrupted.

Both are therefore "the system's fault" some of the time, which is why the two are read together rather than as a clean split between your problem and the platform's.

### Reading them live

```bash
# One application, streamed, with a running aggregate printed on ^C.
aos_timing_report_streamer --application ping

# Everything.
aos_timing_report_streamer
```

```
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
```

That is `ping` from the [ping/pong example](benchmarking.md): a timer firing at 100 Hz that publishes on `/test`, and a watcher for `pong`'s reply on the same channel.

Format is `average [min, max] std stddev`, in **seconds**.

**Read the max column. Ignore the average.** The average tells you about throughput; the max is the only number that has anything to do with whether you meet a deadline. An average of 36 µs with a max of 102 µs is a healthy desktop. An average of 36 µs with a max of 8 ms is a system with a problem you haven't found yet.

`--accumulate` (on by default) keeps a running aggregate across all reports and prints it on `^C`, which is how you get a meaningful max over a long run rather than per-second maxima.

### Reading them from a log

Timing reports are logged like any other channel, so every log you already have contains a full latency history:

```bash
timing_report_dump --application ping /path/to/log
timing_report_dump --stream /path/to/log     # every report, not just the aggregate
```

This is the single most useful thing to do after a bad run. You don't need to reproduce anything.

### What each row tells you

| Row | Field | Meaning |
| --- | --- | --- |
| Watchers | wakeup latency | Handler start − publish timestamp. Includes signal delivery and scheduling. |
| Watchers | handler time | Handler end − handler start, wall clock — your code plus anything that preempted it. |
| Timers | wakeup latency | Handler start − timer deadline. Pure scheduling jitter. |
| Timers | handler time | Same, for a timer callback. |
| Fetchers | latency | Fetch − publish timestamp. Expected to be large for polled loops; that's the point. |
| Senders | size | Message sizes. A growing max often precedes an `OVERWROTE`. |
| Senders | errors | `MESSAGE_SENT_TOO_FAST`, `INVALID_REDZONE`. Non-zero is always a bug. |
| Report | `send_failures` | Reports that couldn't be sent. Non-zero means you are missing data. |

### Interpreting the pattern

Timing reports tell you _which_ application, channel, and metric went wrong. They rarely tell you _why_, because every number in them is an aggregate over a second and most of the interesting causes live outside the process being measured.

So treat the patterns below as a way to narrow down where to point a trace — not as a diagnosis.

- **Timer wakeup latency is bad, handler time is fine, across all applications** — something system-level. IRQs, a non-RT kernel, thermal throttling, another process at a priority it shouldn't have.
- **Timer wakeup latency is bad for one application only** — check its priority against its neighbors with `dump_rtprio`; something above it is likely running long.
- **Handler time max is bad, average is fine** — either a data-dependent path in your code (a rare branch, a `std::map` that grew) or something preempting the handler on those iterations. Timing reports cannot tell these apart, because handler time contains both.
- **Watcher wakeup latency is bad but timer wakeup latency is fine, same application** — points at the wakeup path rather than the scheduler: the sender's fanout loop, or a priority inversion in the wakeup ordering. Check who else subscribes to that channel and at what priority.
- **`MESSAGE_SENT_TOO_FAST`** — the check is on the _spacing_ of consecutive publishes, not the mean rate: it fires when this message would overwrite one sent within the channel's storage duration. That makes it as often a symptom of the publisher's timing being disturbed as of a genuinely over-rate sender — fire late, then early to catch up, and two publishes land too close together while the average rate stays fine. Again, the trace answers which.

In every one of these, the honest next step is the same: arm a trigger, wait for it to fire, and look at what the machine was actually doing.

### Crashes that are really latency problems

Some real-time problems arrive as a crash rather than a number, because AOS deliberately turns silent latency corruption into a loud failure. The ones worth recognizing:

| Crash | What it means |
| --- | --- |
| `Malloced <n> bytes` / `Deleted <ptr>` / `Delete Hook <ptr>` | Something allocated or freed on an RT thread. Look at the backtrace, not the flag that would silence it. |
| `rt_tgsigqueueinfo(...) failed` with `EAGAIN` | `RLIMIT_SIGPENDING` exhausted. This is a **per-user aggregate** across every pending realtime signal on the machine, so the process that crashed is often not the one at fault — some other process of the same user saturated the limit. Raise the limit, or find what is queueing realtime signals. AOS crashes here rather than let a wakeup be dropped and hang a thread indefinitely. |
| `changing to SCHED_FIFO with <n>` | A sender failed to apply the wakeup priority ceiling. Normally means a watcher registered above the sender's `RLIMIT_RTPRIO` — check for a privileged application watching a channel an unprivileged one publishes to. |
| `SIGXCPU` | `RLIMIT_RTTIME`: an RT thread ran 3 seconds of solid CPU without blocking. Almost always a spin loop or a genuinely unbounded computation. |
| `The next message is no longer available ... TOO_OLD` / `OVERWROTE` | A fetcher fell far enough behind that the queue wrapped past it. Either the reader is too slow or the channel's queue is too shallow. The timing report is sent before the crash, so `timing_report_dump` on the log will show you which. |

When reading the timing reports of _other_ applications around one of these, note that a dying process — `CHECK`, `LOG(FATAL)`, or fatal signal — puts itself entirely on `SCHED_OTHER` before it formats anything or symbolizes a backtrace (see [Dying drops to `SCHED_OTHER` first](realtime.md#dying-drops-to-sched_other-first)). So a neighbor's latency spike at the same timestamp is usually a shared cause rather than the crash itself.

---

## Rung 2: `aos_jitter`

Timing reports are aggregated over a second. `aos_jitter` watches a channel and reports the moment a gap between consecutive messages exceeds a threshold.

```bash
# A channel name, plus the message type where the name alone is ambiguous.
# ping publishes every 10 ms, so 15 ms between messages means a cycle slipped.
aos_jitter --max_jitter=0.015 /test aos.examples.Ping

# A channel name with no type watches every type published on that name.
aos_jitter --max_jitter=0.015 /test

# Several channels at once: repeat the name/type pairs.
aos_jitter --max_jitter=0.015 /test aos.examples.Ping /test aos.examples.Pong
```

The channel name must match exactly — there is no wildcard. The message type is optional and matches as a substring, so leaving it off watches every type on that name. With no channel argument at all, `aos_jitter` prints the list of channels and exits.

Things to know:

- **`--max_jitter` is in seconds.** The default of `0.01` is 10 ms.
- It measures the gap between consecutive **publish timestamps** (`context.monotonic_event_time`, stamped by the IPC layer at publish — see [The IPC layer owns the timestamp](realtime.md#the-ipc-layer-owns-the-timestamp)). So it detects _publisher_ jitter, and is unaffected by its own scheduling error.
- **It only measures variation, never slowness.** A publisher that is uniformly late, or a handler that reliably takes 5 ms every cycle, produces perfectly even spacing and trips nothing. `aos_jitter` answers "did the cadence break", not "is this fast enough" — timing reports are where absolute numbers live.

`--print_jitter` is on by default and logs each violation. Printing is not real-time — it's wrapped in `ScopedNotRealtime` so it won't crash, but it allocates and does I/O. For a run where `aos_jitter` itself is RT and must not perturb anything, set `--print_jitter=false`.

`--print_latency_stats` adds a once-per-second percentile dump (25/50/75/90/95/99, plus min/max/mean), which is a good way to watch a distribution drift over a long run.

---

## Rung 3: ftrace, triggered by the violation

This is where root causes are found, and it is what AOS's ftrace integration exists for.

The idea: run ftrace continuously into a ring buffer, which is cheap, and have AOS **stop tracing the instant it detects a timing violation**. The buffer then contains the few hundred milliseconds leading up to the violation, frozen — every context switch, every interrupt, every AOS event. You get to look at exactly what ran instead of your control loop.

The payoff is that it is not an inference. A trace shows the preempting thread by name, the IRQ by number, and the exact instant your handler stopped running — so a question like "was this my code or something else on the box" stops being a judgement call.

### Setting up the trace buffer

`trace-cmd start` configures and enables tracing, then returns — it deliberately does not stay attached or produce a file, which is exactly what you want when the interesting part happens later. Its own man page describes it as "useful just to enable Ftrace and you are only interested in the trace after some event has occurred."

```bash
trace-cmd start -C mono -b 50000 \
  -e sched -e irq -e signal -e timer -e syscalls
```

That is the whole setup, and it composes cleanly with the trigger below: `trace-cmd start` leaves `tracing_on` at 1 and gets out of the way, so when AOS writes `0` to it the buffer freezes with nobody to fight about it. (Use `start`, not `record` — `record` stays attached to manage `tracing_on` and stop the trace itself, which is right when you are driving the capture and wrong when something else is.)

Three things about those options:

- **`-C mono`** is the one people skip and then regret. AOS's trace markers print raw `CLOCK_MONOTONIC` nanoseconds, so without a matching trace clock the kernel timestamps and the AOS timestamps don't line up and correlating them is manual arithmetic.
- **`-b 50000`** sets the per-CPU ring buffer in KB. Bigger buffer, more history before the violation.
- **`-e <subsystem>`** enables whole subsystems rather than hand-picked events, which is the right default here. Picking events means deciding in advance which ones will explain the problem, and the reason you are tracing is that you don't know yet — the one you left off is the one that would have answered it, and you find that out after waiting hours for the next occurrence.

`sched`, `irq`, `signal`, and `timer` cover everything that can take the CPU away from a handler; `sched` answers "what ran instead", and `irq` answers the usual follow-up, which on a `PREEMPT_RT` system is very often an interrupt thread at priority 50. `syscalls` is worth its extra volume: as you back up through the threads looking for a cause, knowing which syscall each was in tells you what it was _doing_ — blocked on a read, in a futex, writing to a device — which is what turns "this thread ran instead of mine" into an explanation. It is by far the highest-volume subsystem of the five, so raise `-b` if the history comes up short.

Without `trace-cmd` installed, the equivalent by hand:

```bash
T=/sys/kernel/tracing            # older kernels: /sys/kernel/debug/tracing
echo 0 > $T/tracing_on
echo nop > $T/current_tracer
echo mono > $T/trace_clock
echo 50000 > $T/buffer_size_kb
for e in sched irq signal timer syscalls; do echo 1 > $T/events/$e/enable; done
echo > $T/trace
echo 1 > $T/tracing_on
```

### Arming the trigger, and the applications

`--enable_ftrace` (default false, `aos/ftrace.cc`) gates every AOS trace marker. Nothing is written to `trace_marker` unless you ask, which keeps it free in production.

```bash
# Watchdog: freeze the trace the moment ping slips a cycle.  ping runs at RT
# priority 5, so 1 keeps this below it -- see the note on priority below.
aos_jitter --enable_ftrace --max_jitter=0.015 --priority=1 /test aos.examples.Ping

# Or watch several channels, so any of them slipping freezes the trace.
aos_jitter --enable_ftrace --max_jitter=0.015 --priority=1 \
  /test aos.examples.Ping /test aos.examples.Pong
```

When it trips, `aos_jitter` writes a marker naming the channel and the observed gap, then writes `0` to `tracing_on`.

Run the watchdog on the RT scheduler at a priority **below everything under test**. Put it above them and it preempts what it is watching, so it reports jitter it caused; leave it on `SCHED_OTHER` and the load you are chasing can starve it until the buffer has wrapped past the evidence. Below the applications but above the non-RT noise bounds the delay without perturbing anything. Some lag between violation and freeze is expected — size `buffer_size_kb` to cover it.

**That stop is global.** `tracing_on` is one switch for the whole machine, not something scoped to the process that wrote it — so a single `aos_jitter` noticing that one channel slipped a cycle freezes the buffer for _everything_: every other application's markers, every context switch, every interrupt, on every CPU. This is what makes it a smoking gun rather than a data point. You get the moment of the violation with the entire system's behavior around it, from one process watching one symptom, which is exactly the evidence that is otherwise impossible to catch.

So one `aos_jitter`, given the handful of channels whose cadence you actually care about, becomes a watchdog for the whole system: any of them missing a beat stops the trace, and you come back later to a frozen picture of whatever caused it.

**Turn `--enable_ftrace` on for the applications you are investigating too, not just the watchdog.** Every AOS event loop emits markers for its own watchers, timers, sends, and fetches, and those markers are what let you orient yourself in a trace: they tell you where each handler began and ended, and which channel each one belongs to. Without them you get a wall of `sched_switch` naming threads, and the work of mapping that back onto "which callback was this, and what was it supposed to be doing" is yours. With them, the AOS events sit on the same timeline as the kernel events and the trace reads as a story about your system rather than about the scheduler.

Enable it the same way you set any other flag. Under `starter`, add it to the application's `args` in the config:

```json
{"name": "ping", "args": ["--enable_ftrace"]}
```

For turning it on across many applications at once, abseil's `--flagfile` and `--tryfromenv` both work, so a shared flag file or an environment variable saves editing every entry.

There is a real cost, which is why this is opt-in: each marker is a `write(2)` to `trace_marker` on the event loop's thread, on the RT path. It is bounded and small, but it is not nothing, so enable it while you are chasing something and turn it back off afterwards.

`starterd` also freezes the trace on `SIGCHLD` when run with `--enable_ftrace`, so an application dying unexpectedly stops the buffer too — usually more informative than the core dump.

### Collecting it, and reading it in KernelShark

Once the trigger has fired, pull the frozen buffer out and open it:

```bash
trace-cmd extract -o /tmp/violation.dat
kernelshark /tmp/violation.dat

trace-cmd reset                  # when you are done tracing entirely
```

`extract` reads the kernel ring buffer into a `trace.dat`, and is meant to be used exactly this way — after a `start`, once tracing has stopped, no matter who stopped it.

**Start with KernelShark rather than the text.** A timing violation is a question about what several CPUs were doing at once over a few hundred microseconds, and that is a picture, not a log. KernelShark draws a per-CPU timeline with each thread as a band, so a preempting IRQ thread, a thread blocked in a syscall, or an unexpected migration is something you see at a glance instead of something you reconstruct by reading timestamps in order. The AOS markers land on that same timeline, which is what makes the application's story and the kernel's line up.

The text dump is still there when you want to grep it or paste it into a bug:

```bash
cat /sys/kernel/tracing/trace > /tmp/violation.txt
```

Read that one from the end, where the AOS marker that tripped the trigger sits, working upwards.

### What AOS already puts in the trace

Given `--enable_ftrace`, you do not need to add any instrumentation of your own to see the shape of an AOS system in a trace. `aos/events/event_loop_tmpl.h` emits markers on every event loop operation:

| Marker | Emitted when |
| --- | --- |
| `<channel>: watcher start: now=… event=… queue=…` | A watcher callback begins |
| `<channel>: watcher end: now=…` | It returns |
| `timer: <name>: start now=… event=…` | A timer callback begins |
| `timer: <name>: end now=…` | It returns |
| `phased: <name>: start now=… event=… cycles=…` | A phased loop begins; `cycles` is the number of periods elapsed since it last ran, so anything above 1 means periods were missed |
| `phased: <name>: end now=…` | It returns |
| `<channel>: sent internal\|external\|shared: event=… queue=…` | A message was published |
| `<channel>: fetch next\|latest: now=… event=… queue=…` | A fetch returned a message |
| `<channel>: fetch next\|latest: still event=… queue=…` | A fetch found nothing new |

`event` is the publish timestamp or timer deadline; `now` is when the handler actually started. **`now` − `event`, read straight off the trace, is the wakeup latency for that single event** — the same quantity timing reports aggregate, but for the specific late event you are investigating.

Message bridge adds its own, so a cross-node latency problem is traceable end to end:

| Marker | Meaning |
| --- | --- |
| `Bridge sent channel=… peer=… queue=… event=… size=… ttl_ms=…` | Server forwarded a message to a peer |
| `Bridge skipping channel=… peer=… queue=…` | Already sent, or peer not connected |
| `Bridge disconnected channel=… peer=… queue=…` | Send failed |
| `Bridge message size=… remote=… transmit=… remote_queue=… channel=…` | Client received and republished |
| `Bridge duplicate message size=…` | Client discarded a duplicate |

### Reading a trace

The recipe, with the capture open in KernelShark. Steps 1 and 2 locate the moment; from there it is a matter of looking at which band is filled on the CPU in question, rather than grepping `sched_switch` by hand:

1. Find the AOS marker that tripped the trigger. Note its timestamp.
2. Find the `watcher start` or `timer: … start` for the late event. The gap between `event=` and `now=` in that marker is the latency you're explaining.
3. Walk backwards through `sched_switch` on that CPU. What was `prev_comm`/`next_comm` during the gap?
4. If the answer is an `irq/N-something` thread or `ksoftirqd`, you have an interrupt problem — go to `irq_affinity` config (see [How AOS is Real-Time](realtime.md#irqs-and-kernel-threads)).
5. If the answer is another AOS application, you have a priority assignment problem — `dump_rtprio` will show you the ordering.
6. If the answer is your own process, but not the thread you expected, you have a threading problem inside the application.
7. If nothing was running and the CPU was idle, the delay is in wakeup delivery, not scheduling. Look at the `signal_generate` → `signal_deliver` pair, and at whether the sender was boosted.
8. If `sched_migrate_task` shows up, your affinity isn't pinned and you're paying for cache-cold restarts.

### Adding your own triggers

The same mechanism is available inside any application. Do this for any condition you can detect cheaply that means "something is wrong":

```cpp
#include "aos/ftrace.h"

class MyLoop {
 public:
  MyLoop(aos::EventLoop *event_loop) : event_loop_(event_loop) {
    event_loop_->MakeWatcher("/sensors", [this](const Sensors &s) {
      const auto start = event_loop_->monotonic_now();
      DoWork(s);
      const auto elapsed = event_loop_->monotonic_now() - start;

      if (elapsed > std::chrono::milliseconds(2)) {
        // Both calls are RT-safe: a bounded snprintf and two write(2)s to
        // already-open fds.  Both are silent no-ops without --enable_ftrace.
        ftrace_.FormatEvent("MyLoop overran: %" PRId64 " ns", elapsed.count());
        ftrace_.TurnOffOrDie();
      }
    });
  }

 private:
  aos::EventLoop *event_loop_;
  aos::Ftrace ftrace_;
};
```

`Ftrace` opens its fds in the constructor (non-RT) and does nothing but `write(2)` afterwards, so `FormatEvent` is safe from an RT handler. Construct it at startup, never in the handler. If `--enable_ftrace` isn't set the fds are never opened and both calls return immediately, so leaving this in production code costs a branch.

`TurnOffOrDie()` will `CHECK`-fail if `tracing_on` couldn't be opened — that's deliberate, so a trigger you _thought_ was armed doesn't silently do nothing.

---

## Rung 4: ping/pong

When you want to know whether the platform itself can do what you are asking, before pointing any of the above at your own application.

`ping` and `pong` are the smallest complete AOS system: a 100 Hz timer publishing on `/test`, and a watcher replying on the same channel. Because there is essentially nothing in the handlers, what the timing reports show is the floor — the platform's scheduling and wakeup cost, with none of your code mixed in.

```bash
# Terminal 1 and 2
bazel run -c opt //documentation/examples/ping_pong:ping
bazel run -c opt //documentation/examples/ping_pong:pong

# Terminal 3
aos_timing_report_streamer --application ping
```

Run it on a new board before trusting any number from it, and again whenever a result on a real application looks impossibly bad — if ping's wakeup latency is also bad, the problem is under you rather than in your application. It composes with everything above: point `aos_jitter` at `/test`, arm the ftrace trigger, and you have a self-contained reproduction that does not require your system to misbehave first.

See [Benchmarking](benchmarking.md) for a worked example with reference numbers.

---

## Make it happen more often: load

The expensive part of an RT bug is waiting for it. Load compresses that wait, and it is not cheating — real systems are loaded. Logging flushes, a filesystem sync, a network burst, a cron job waking up to rotate logs or rebuild an index, someone sshing in to look at the problem. On the timescales that matter here, a tenth of a second of background work is an ordinary event, not an exotic one. A system that only meets its deadlines on an idle box does not meet its deadlines.

So put load on deliberately, and run the whole ladder above underneath it. `aos_jitter` with the ftrace trigger armed, under a stressor, will often catch in minutes what takes hours on a quiet machine.

**The load has to be realistic, though, and that is a real constraint.** It is trivial to build load that breaks anything: start a `SCHED_FIFO` priority 99 spinner and every deadline on the machine will be missed.

The line is the RT scheduler itself. Anything running at RT priority above your applications is _your_ configuration — if something is up there that shouldn't be, that is a misconfiguration to fix, not a hazard to survive, and simulating it proves nothing. **Everything below that line is fair game, and all of it is realistic**, because you do not get to choose what non-RT work lands on the machine, how much of it there is, or when it arrives. So make the stressors as brutal as `SCHED_OTHER` permits, and treat any deadline missed under them as a genuine bug in your system rather than an unfair test.

```bash
# CPU contention.
stress --cpu $(nproc) --timeout 600s

# The same, as aggressive as a non-RT task can be.
nice -n -20 stress --cpu $(nproc) --timeout 600s

# Memory bandwidth and cache pressure.  --vm-keep redirties instead of
# reallocating, so this hammers bandwidth rather than the allocator;
# --vm-stride 64 touches one byte per cache line.
stress --vm 4 --vm-bytes 512M --vm-keep --vm-stride 64 --timeout 600s

# I/O, which is also how you generate softirq work.
stress --io 4 --hdd 2 --timeout 600s

# Bursty rather than steady.  A low duty cycle spread over many threads is
# far nastier than the same total load in a few: 100 threads at 1% each
# wake constantly, and CFS keeps treating every one of them as interactive.
stress-ng --cpu 100 --cpu-load 1 --cpu-load-slice 10 --timeout 600s
```

**Prefer load that doesn't make syscalls.** `stress --cpu` spins on `sqrt()` entirely in userspace, and `--vm ... --vm-keep` redirties memory it already owns, so neither of them shows up in the trace. `--io` and `--hdd` spin on `sync()` and `write()`/`unlink()`, which with `-e syscalls` enabled will bury the events you actually care about and burn the ring buffer you sized for history. Reach for those when softirq or block-layer behavior is the thing under investigation; otherwise keep the stressors quiet so the trace stays readable.

For bursty load specifically, `stress-ng --cpu-load P --cpu-load-slice N` gives you a duty cycle: `P` percent loading, in busy slices of `N` milliseconds. If you need real control over the shape — periods, phases, per-thread policies — [`rt-app`](https://github.com/scheduler-tools/rt-app) is the tool the kernel scheduler people use for exactly this, and takes a JSON description of threads with run/sleep phases.

What that load actually does to you:

- **Nice it to -20.** A `SCHED_OTHER` task can never preempt an RT thread, which makes it tempting to assume it cannot hurt. It can: it takes every cycle the RT threads leave, and it competes for everything underneath the scheduler — locks, memory bandwidth, cache, the page cache. `-20` is the most aggressive a non-RT task can be, which is exactly what you want from a stressor.
- **Memory and cache pressure hurt in a way CPU pressure doesn't.** CPU contention shows up as scheduling delay: visible, expected, and easy to find in a trace. Memory bandwidth contention doesn't delay your thread at all — it makes your thread _slower_, because every cache miss now costs more. A memory-intensive RT handler can take substantially longer in wall clock with no scheduling event anywhere in the trace to explain it. This is the load that produces the frustrating "handler time is bad and nothing preempted it" case.
- **Memory pressure also finds every gap in a processing chain.** If A triggers B, and there is even a small gap between A finishing and B starting, that gap is long enough for something else to be scheduled and evict your working set. B then runs cache-cold: same code, same inputs, several times the wall-clock cost. Under production loads this has been worth **40–60 ms of extra latency**, which is enormous next to anything else in this document. The more stages a pipeline has, the more of these windows exist, and none of them are visible as a scheduling delay — the cost lands inside the next handler, one stage downstream of where the disturbance happened.
- **Bursty load is worse than steady load.** CFS deliberately favors tasks it judges interactive — ones that sleep and wake rather than spin — so a stressor running flat out gets treated as a batch hog and throttled, while one that wakes, does a little, and sleeps keeps getting handed the CPU promptly. The realistic background work on your machine is the bursty kind. Test both.

## Interference from inside your own process

A process is not real-time because one of its threads is. Everything else in it shares an address space with the RT thread, and most of what it can do reaches across:

- **Shared locks.** The allocator is the obvious one: a `SCHED_OTHER` thread holding the malloc arena lock when the RT thread allocates is a textbook priority inversion with no IPC anywhere in it, which is most of what `--die_on_malloc` is really protecting you from (see [The malloc hooks](realtime.md#the-malloc-hooks)). Any other lock the two threads share behaves the same way, and so does anything with a hidden lock inside it.
- **Page faults are a process-wide resource.** A non-RT thread touching new memory can hold `mmap_lock` while the RT thread's own fault waits behind it. `mlockall` keeps the RT thread from faulting; it does nothing about its siblings making it wait.
- **Softirqs run in the thread that caused them.** On `PREEMPT_RT`, when a thread re-enables bottom halves from preemptible context with softirq work pending, `__local_bh_enable_ip()` calls `__do_softirq()` **inline — in that thread, at that thread's priority** (`kernel/softirq.c`). It only defers to `ksoftirqd` when bottom halves are re-enabled from non-preemptible context, which in practice means softirqs raised by hard IRQ handlers. So a worker doing network or block I/O is not merely _generating_ work for some kernel thread to run later; it is running that work itself. Whatever priority that thread has is the priority the kernel's networking or block work now gets. The `ksoftirqd` share is real too, and its priority is yours to set (see [IRQs and kernel threads](realtime.md#irqs-and-kernel-threads)) — but it is not where most of this lands.
- **Don't do non-RT I/O from an RT thread.** This is old guidance that has aged well, though the specifics have moved. Descriptor _lookup_ is lock-free RCU these days, so the classic "every `read()` takes the fd table lock" worry is gone. What remains is worth respecting: `files->file_lock` still serializes allocating and closing descriptors, and `expand_fdtable()` calls `synchronize_rcu()` when the descriptor table has to grow and the `files_struct` is shared — a full RCU grace period, inside your `open()`. Separately, `read()`/`write()` take `f_pos_lock`, a **mutex** in `struct file`, whenever the descriptor is shared and the file position is therefore contended. Two practical rules fall out: open every descriptor you need before going RT, so you never grow the table on the RT path; and if an fd really is shared, use `pread`/`pwrite`, which carry an explicit offset and skip `f_pos_lock` entirely.
- **Block I/O gets submitted when the submitting thread blocks.** The block layer batches requests into a plug hanging off `task_struct`, and `schedule()` flushes it through `sched_submit_work()` when the task goes to sleep. Because the plug is per-thread, that work lands on the thread that submitted the I/O rather than on a sibling — but it lands at a moment nobody chose, inside whatever call happened to block.

Those are all cases where a sibling thread's behavior reaches you. The next two are worse, because they are properties of the **address space**, which threads cannot opt out of sharing — no priority, and no PI mechanism, helps:

- **Signals are aimed at the process, and any thread can take one.** A process-directed signal (an ordinary `kill(pid, ...)`) is delivered to whichever thread happens to have it unblocked — the kernel picks, and you do not. So a signal meant for "the application" can land on, and interrupt, the RT thread. Blocking is per-thread and inherited by threads created afterwards, which makes the disposition depend on thread creation order, and the default action for a realtime signal is to kill the whole process. AOS sidesteps this for its own wakeups by sending them **thread-directed** with `rt_tgsigqueueinfo` and keeping `kWakeupSignal` blocked in every thread but the receiver's, draining it through a `signalfd` instead (`aos/ipc_lib/thread_signal_linux.cc`) — the header notes that on macOS, where cross-process thread-directed signals do not exist, the fallback to a process-directed `kill()` really does cause spurious wakeups on other threads. Nothing protects you from signals AOS doesn't own.
- **`mmap_lock` is per-process.** Any thread that calls `mmap`, `munmap`, `mprotect`, or `brk` takes it for write, excluding everyone; any thread taking a page fault takes it for read. So a non-RT sibling doing ordinary memory work can block your RT thread's fault, and the worst version is a sibling taking a _major_ fault — it holds the lock across the I/O. Recent kernels added per-VMA locks that handle most faults without `mmap_lock` at all, which helps a great deal, but the write side still excludes everything and the fallback paths still exist.
- **TLB shootdowns are per-address-space, and they are interrupts.** `mm_struct` tracks which CPUs have run threads of the process, and unmapping or changing protections sends an IPI to every one of them. An IPI is a hardware interrupt: it lands on your RT thread's core and runs, no matter what priority anything has. Published measurements put IPI latency around 1–5 µs per core, which is small until a many-core machine makes it not — a single `munmap` on a 120-core box has been measured at over 120 µs, most of it shootdown.

**Some of this cannot be fixed inside a process.** Priorities, PI, and careful lock discipline all work on things threads choose to share. An address space is not one of those — a non-RT thread that merely allocates and frees will fire IPIs at its RT siblings and contend for `mmap_lock` with them, and nothing you can tune changes that. When you need real isolation from that class of interference, you need a separate _process_, not a separate thread. That is a large part of why AOS pushes work across processes and gives you an IPC layer good enough to make it painless (see [Layer 5](realtime.md#layer-5-the-event-loop)).

These have a useful signature in a trace: **the preempting thread has the same PID as the victim.** If `sched_switch` shows another thread of your own process, retuning priorities against other applications will not help — the problem is inside the application. That is also the case where `syscalls` events earn their volume, since they tell you what the offending sibling was actually doing.

The fix is the same as it is between processes: don't let the RT thread share anything unbounded with the non-RT ones, and give every worker thread an explicit priority instead of whatever it inherited. See [Threading](threading.md).

## Auditing the system

### Priorities

```bash
dump_rtprio --config aos_config.json
```

Output is CSV: `exe,name,cpumask,policy,nice,priority,tid,pid,ppid,sid`, sorted by realtime priority. Every thread on the system, not just AOS ones.

This is what you run first when one application's wakeup latency is bad. Things to look for:

- An AOS application at a higher priority than you intended.
- `irq/*` threads (default priority 50 on `PREEMPT_RT`) above your control loops.
- `ksoftirqd` competing with RT work.
- A vendor daemon that quietly runs `SCHED_FIFO`.
- Threads with a `cpumask` of all cores that should be pinned.
- A worker thread inside an application that didn't get the priority its config specified.

Fix what you find in the application config (`priority`, `scheduling_policy`, `cpu_affinity`) and the IRQ config (`aos/starter/kthread.fbs`), not in code.

### Affinity and pinning

Pinning is the strongest tool here and the easiest to overuse.

What it is good at is **segregation**. Steering a device's interrupts to a core that runs nothing critical removes that work from your control loop's core entirely, and priority tuning cannot achieve the same thing — the hard IRQ handler runs when the interrupt fires, whatever your threads are doing. Separating devices from each other matters too: NAPI polls per core rather than per device, so two interfaces sharing a core share one poll loop with no priority between them (see [NAPI is per-core, not per-device](realtime.md#napi-is-per-core-not-per-device)). Giving latency-critical traffic its own core, or turning on threaded NAPI, is how you get that ordering back.

The failure mode is pinning everything. Every pin you add is a constraint the scheduler can no longer use to route around a busy core, so a system pinned everywhere often has _worse_ worst-case latency than one pinned in a few deliberate places.

It is also brittle in two directions. A pinning scheme encodes a core count and a device layout, so it breaks on the next board revision or a machine with a different topology — and it encodes your _workload_, which is worse, because that changes constantly. Add an application, move work between processes, change a rate, and the assignment that was carefully balanced is now carefully wrong. Neither failure announces itself; both show up as latency nobody can explain, and re-deriving a pinning layout from scratch every time the system evolves is miserable work that tends not to get done.

So: **pin the minimum.** Generally that means the critical path and nothing else. And pin in response to a measurement, not in anticipation — you should be able to point at the trace showing the interference and say what was interfering before you add a constraint to prevent it. A pin you cannot explain is a pin nobody will dare remove later.

### RT throttling

Rare, but the symptom is unlike anything else in this document. The kernel caps how much of each period RT tasks are allowed to consume, and when they exceed it, it **stops running them** and lets `SCHED_OTHER` work through:

```bash
cat /proc/sys/kernel/sched_rt_period_us     # 1000000 -- a 1 s window
cat /proc/sys/kernel/sched_rt_runtime_us    # 950000  -- of which RT may use 0.95 s
```

Those defaults mean that if the RT tasks on a CPU collectively use more than 95% of a second, the rest of that second is taken away from them. Not from the offending task — from _all_ of them.

It is miserable to debug because the symptom looks like nothing at all. Your control loop simply stops running for up to 50 ms, and the trace shows no higher-priority thread, no interrupt, no lock, nothing preempting it. Everything you would normally look for is absent, because the scheduler is not choosing something else over you; it has removed you from consideration.

The kernel does tell you, in the log:

```bash
dmesg | grep -i 'RT throttling'      # sched: RT throttling activated
```

Check there first when the trace comes up empty — but note the catch: **it is a warn-once message.** The kernel prints it the first time throttling engages and stays silent forever after, so its absence on a box that has been up for a week means nothing at all. A clean `dmesg` is not evidence you are not being throttled.

Two things follow. First, if you see an unexplained gap of tens of milliseconds with nothing running above you, check these values and check whether your RT threads are near the budget. Second, a runaway RT thread that spins will burn the budget for every other RT thread sharing the CPU, so one application's bug appears as every application's problem.

Throttling can be turned off entirely with `echo -1 > /proc/sys/kernel/sched_rt_runtime_us`. **Don't.** It is tempting on a dedicated target — you own the machine, why should the kernel take 5% of it — but the headroom is not decoration. Kernel workqueues, RCU callbacks, and assorted deferred cleanup all need to run, and starving them does not fail politely: work backs up, and the system degrades in ways that are hard to attribute to the RT thread that caused them. Worse, the throttle is what leaves you a shell. With it off, a runaway RT thread means you cannot log in to find out what happened, which turns a debuggable problem into a power cycle.

If 95% is genuinely not enough headroom for your RT work, the RT work is the thing to fix.

This is a different mechanism from `RLIMIT_RTTIME` (see [How AOS is Real-Time](realtime.md#memory-locking)), which kills a single thread that runs too long rather than pausing all of them.

### Interrupts

```bash
watch -n1 'cat /proc/interrupts'
```

Watch which core is absorbing each interrupt source. A core that is taking a rising IRQ count and also running a control loop is the problem, and `irq_affinity` is the fix.

### CPU usage

`aos/util/top.h` provides per-process and per-thread CPU and memory sampling from within an AOS application, which is how `dump_rtprio` and system monitors get their numbers. A thread that's pegged at 100% is a different problem from one that's blocked — `aos::GetCurrentThreadCpuTime()` distinguishes them.

---

## Catching violations before they ship

The best latency bug is one that never reaches hardware.

### Let the malloc hooks work

`--die_on_malloc` is on by default and crashes on any allocation from an RT thread. Do not turn it off to make a test pass. If a crash points at a library you don't control, the fix is to pre-allocate before the RT transition, or to wrap the genuinely-non-RT section in `aos::ScopedNotRealtime` with a comment explaining why it's acceptable.

The flags that disable RT enforcement exist for desktop and CI convenience, and should not appear on a deployed target:

| Flag | Effect | Legitimate use |
| --- | --- | --- |
| `--die_on_malloc=false` | Allows allocation while RT | Debugging only |
| `--skip_realtime_scheduler` | Never actually goes `SCHED_FIFO` | Desktops without RT privileges |
| `--skip_locking_memory` | No `mlockall` | Desktops without `RLIMIT_MEMLOCK` |

### Assert it in tests

The RT-mode flag is thread-local state, not a scheduler query, so it behaves identically in simulation. That means RT constraints are testable on a desktop:

```cpp
// Fail the test if this is ever reached from an RT context.
aos::CheckNotRealtime();

// Or assert we're on the RT path where we expect to be.
aos::CheckRealtime();
```

`SimulatedEventLoopFactory` marks handlers realtime the same way `ShmEventLoop` does, so a `malloc` in a handler that would crash on the target crashes in the unit test.

---

## Quick reference

```bash
# Live latency, one application, with aggregate on ^C
aos_timing_report_streamer --application NAME

# Latency history from a log you already have
timing_report_dump --application NAME /path/to/log

# Watchdog on a channel; --max_jitter is SECONDS
aos_jitter --max_jitter=0.015 /test aos.examples.Ping

# Same, but freeze the ftrace buffer on violation.  Give the applications
# under investigation --enable_ftrace too, or the trace has no AOS markers.
# --priority goes BELOW everything under test, so the watchdog never preempts it.
aos_jitter --enable_ftrace --max_jitter=0.015 --priority=1 /test aos.examples.Ping

# Every thread's priority, policy, and affinity
dump_rtprio --config aos_config.json

# Kernel RT floor
cyclictest -m -p 80 -i 200 -d 0 -t -a -h 400

# RT throttling budget -- an unexplained ~50 ms stall with nothing above you
cat /proc/sys/kernel/sched_rt_period_us /proc/sys/kernel/sched_rt_runtime_us

# Set up tracing, then leave it running for the trigger to freeze
trace-cmd start -C mono -b 50000 -e sched -e irq -e signal -e timer -e syscalls

# Collect and read a frozen trace
trace-cmd extract -o /tmp/violation.dat && kernelshark /tmp/violation.dat
```

---

## Further reading

- [How AOS is Real-Time](realtime.md) — the mechanisms these tools measure
- [Benchmarking](benchmarking.md) — worked ping/pong example with reference numbers
- [Threading](threading.md) — worker threads and their priorities
- `documentation/adr/0001-aio-io-uring-single-issuer.md` — event loop backend and timer behavior
- `aos/ftrace.h` — the trace marker API
- `aos/events/event_loop.fbs` — the timing report schema
