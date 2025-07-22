import asyncio
import enum
from collections import OrderedDict
from collections.abc import Callable, Coroutine
from typing import Any, Dict, List, Optional, Tuple, Type

from aos.events.event_loop_c import ffi, lib


class _FutureState(enum.Enum):
    PENDING = "PENDING"
    FINISHED = "FINISHED"


class FutureAlreadyDoneError(Exception):
    pass


class FutureResultNotReadyError(Exception):
    pass


class Future:
    """A simple Future with a binary state.

    When its result is set, all registered callbacks are immediately invoked.
    """

    def __init__(self) -> None:
        self._state: _FutureState = _FutureState.PENDING
        self._result: Any = None
        self._callbacks: List[Callable[[], None]] = []

    def __await__(self) -> Any:
        if not self.done():
            yield self
        return self.result()

    def done(self) -> bool:
        return self._state != _FutureState.PENDING

    def add_done_callback(self, fn: Callable[[], None]) -> None:
        if self.done():
            fn()
        else:
            self._callbacks.append(fn)

    def set_result(self, result: Any = None) -> None:
        if self.done():
            raise FutureAlreadyDoneError()
        self._result = result
        self._state = _FutureState.FINISHED
        self.__invoke_callbacks()

    def result(self) -> Any:
        if not self.done():
            raise FutureResultNotReadyError()
        return self._result

    def __invoke_callbacks(self) -> None:
        if not self.done():
            raise FutureResultNotReadyError()
        for callback in self._callbacks:
            callback()
        self._callbacks.clear()


class Timer:
    """An async timer that wraps a C AOS Timer.

    Once scheduled, its tick() can be awaited to resume execution at the timer's next
    tick (expiration).
    """

    def __init__(self, runtime: "EventLoopRuntime") -> None:
        if runtime.is_running():
            raise RuntimeError()
        self._runtime = runtime
        self._handle = ffi.new_handle(self)
        self._c_timer_handle = self._runtime._loop().add_timer(
            self._runtime._loop(), Timer._timer_callback, self._handle)
        self._on_tick: Future = None
        # The task in which the timer was added.
        self._task: "Task" = self._runtime._get_current_task()
        if not self._task:
            raise RuntimeError()

    # TODO(Sanjay): Do better with timepoints and durations. Nanoseconds since epoch is
    # too primitive.
    def schedule(self, start: int, period: int = 0) -> None:
        """Schedule the timer.

        Args:
            start (int): Nanoseconds since epoch on the monotonic clock at which the
                timer will expire.
            period (int): If specified, nanoseconds between period ticks of the timer.
                If unspecified or set to 0, the timer will tick just once at `start`.
        """
        self._c_timer_handle.schedule(self._c_timer_handle, start, period)

    def cancel(self) -> None:
        """Cancel the timer, if scheduled."""
        self._c_timer_handle.disable(self._c_timer_handle)

    async def tick(self) -> None:
        """Yield till the timer's next tick."""
        # TODO(Sanjay): Check if the timer was scheduled. We probably don't want the
        # user awaiting a tick that may never happen.
        self._on_tick = Future()
        self._on_tick.add_done_callback(self.__resume_on_tick)
        await self._on_tick

    def __resume_on_tick(self) -> None:
        if not self._on_tick.done():
            raise FutureResultNotReadyError()
        try:
            self._runtime._set_current_task(self._task)
            self._task.resume()
        except StopIteration:
            pass
        finally:
            self._runtime._set_current_task(None)

    @staticmethod
    @ffi.callback("void(void *)")
    def _timer_callback(timer_handle) -> None:
        timer = ffi.from_handle(timer_handle)
        if not timer._on_tick or timer._on_tick.done():
            # The timer was scheduled but not awaited.
            return
        timer._on_tick.set_result()


class Channel:
    """This class represents a unique AOS channel.

    An AOS channel is defined by the combination of its name and its type. You can have
    unique channels with the same name and different types, or vice versa.
    """

    def __init__(self, channel_name: str, channel_type_cls: Type[Any]) -> None:
        self._name: str = channel_name
        self._type_cls: Type[Any] = channel_type_cls
        self._type: str = self.__get_fully_qualified_type(channel_type_cls)
        self._uid: str = "-".join([self._name, self._type])

    def __hash__(self) -> int:
        return hash(self._uid)

    def __eq__(self, other) -> bool:
        if not isinstance(other, Channel):
            return False
        return self._uid == other.channel_uid()

    def name(self) -> str:
        """Return the channel's name."""
        return self._name

    def type(self) -> str:
        """Return the channel's namespace-qualified type as a string."""
        return self._type

    def channel_uid(self) -> str:
        """Return a unique identifier for this channel.

        This is just a hyphen-separated concatenation of its name and type, which is
        guaranteed to be unique.
        """
        return self._uid

    def parse(self, buffer):
        """Parse the buffer as the channel's type and return it."""
        return self._type_cls.GetRootAs(buffer, 0)

    def __get_fully_qualified_type(self, channel_type_cls: Type[Any]) -> str:
        # Each flatbuffers schema compiled for Python is a module that follows the
        # naming pattern "<filename>_fbs_py". The submodule hierarchy within it mirrors
        # the namespaces of the objects in the schema and its includes. We're only
        # interested in the namespace-qualified name, so remove the initial bit.
        module = channel_type_cls.__module__
        if "_fbs_py." in module:
            return module.split("_fbs_py.", 1)[1]
        else:
            raise ValueError(
                f"Unexpected module name: {module}. Are you sure this is flatbuffers generated object?"
            )


class Fetcher:
    """A fetcher that wraps a C AOS Fetcher and a C AOS Watcher.

    The watcher is "lazy" in the sense that when a new message arrives on the channel,
    it only does something when at least one task awaits `next`.

    The `fetch*` methods can be used to poll the channel for a message.
    """

    def __init__(
        self,
        runtime: "EventLoopRuntime",
        channel: Channel,
    ) -> None:
        if runtime.is_running():
            raise RuntimeError()
        self._runtime = runtime
        self._handle = ffi.new_handle(self)
        self._channel = channel
        c_channel_name = ffi.new("char[]", channel.name().encode("utf-8"))
        c_channel_type = ffi.new("char[]", channel.type().encode("utf-8"))
        self._runtime._loop().make_watcher(
            self._runtime._loop(),
            c_channel_name,
            c_channel_type,
            Fetcher._watcher_callback,
            self._handle,
        )
        self._c_fetcher = self._runtime._loop().make_fetcher(
            self._runtime._loop(), c_channel_name, c_channel_type)
        self._fetched_message = None
        # Multiple tasks can be associated with a fetcher. When a task awaits this
        # fetcher's next(), it gets tracked here and is resumed when a new message
        # arrives on the channel.
        self._tasks: OrderedDict["Task", bool] = OrderedDict()
        self._on_message: Future = None
        self._last_message: Any = None

    def register_task(self, task: "Task") -> None:
        """Register a task with this fetcher.

        Only registered tasks can await `next`. It's how this class tracks what tasks to
        poke when a new message shows up on the channel.
        """
        if self._runtime.is_running():
            raise RuntimeError()
        self._tasks[task] = False

    # TODO(Sanjay): Can we do better than Any for the return type?
    def fetch(self) -> Tuple[bool, Any]:
        """Fetch the latest message on the channel without blocking.

        Returns:
            bool: True if a new message was fetched.
            Any: The last message that was fetched.
        """
        return (self._c_fetcher.fetch(self._c_fetcher), self.__get())

    # TODO(Sanjay): Can we do better than Any for the return type?
    def fetch_next(self) -> Tuple[bool, Any]:
        """Fetch the next message on the channel without blocking.

        A channel is a message queue, and "next" means the message that arrived
        immediately after the last one that was fetched. Beware, the queue size is
        finite, and if new messages arrive faster than you call `fetch_next`,
        eventually you will fall behind and this method will fail.

        Returns:
            bool: True if a new message was fetched.
            Any: The last message that was fetched. If accompanied with True, this is
                the next message in the queue. Otherwise, we're all caught up.
        """
        return (self._c_fetcher.fetch_next(self._c_fetcher), self.__get())

    # TODO(Sanjay): Can we do better than Any for the return type?
    async def next(self) -> Any:
        """Yield till the next message arrives on the channel.

        Returns:
            Any: The new message that just arrived.
        """
        if not self._on_message or self._on_message.done():
            self._on_message = Future()
            self._on_message.add_done_callback(self.__resume_on_message)
        # Track that the current task is awaiting next(), so when the next message
        # arrives we know to resume it.
        task = self._runtime._get_current_task()
        if task not in self._tasks:
            raise RuntimeError()
        self._tasks[task] = True
        await self._on_message
        return self._channel.parse(self._last_message)

    def __get(self) -> Any:
        context = self._c_fetcher.context(self._c_fetcher)
        if not context.data:
            return None
        self._fetched_message = ffi.buffer(
            ffi.cast("const char *", context.data), context.size)
        return self._channel.parse(self._fetched_message)

    def __resume_on_message(self) -> None:
        if not self._on_message.done():
            raise FutureResultNotReadyError()
        # Store the message separately. When tasks are resumed, if one awaits `next`
        # again, it will reset `_on_message`. All following tasks will then lose access
        # to the message.
        self._last_message = self._on_message.result()
        for task, resume in self._tasks.items():
            if not resume:
                continue
            try:
                self._runtime._set_current_task(task)
                # Untrack this task. When we resume it, if it awaits this fetcher, it
                # will get tracked again.
                self._tasks[task] = False
                task.resume()
            except StopIteration:
                pass
            finally:
                self._runtime._set_current_task(None)

    @staticmethod
    @ffi.callback("void(const context_t *, const void *, void *)")
    def _watcher_callback(context, message, fetcher_handle) -> None:
        fetcher = ffi.from_handle(fetcher_handle)
        if not fetcher._on_message or fetcher._on_message.done():
            # The fetcher wasn't awaited since the last time we got a message.
            # TODO(Sanjay): The last message is no longer valid. How should we handle that?
            return
        fetcher._on_message.set_result(
            ffi.buffer(ffi.cast("const char *", context.data), context.size))


class Sender:
    """A sender that wraps a C AOS Sender.

    It sends a serialized buffer on the channel by copying it.
    """

    def __init__(
        self,
        runtime: "EventLoopRuntime",
        channel: Channel,
    ) -> None:
        if runtime.is_running():
            raise RuntimeError()
        c_channel_name = ffi.new("char[]", channel.name().encode("utf-8"))
        c_channel_type = ffi.new("char[]", channel.type().encode("utf-8"))
        self._c_sender = runtime._loop().make_sender(runtime._loop(),
                                                     c_channel_name,
                                                     c_channel_type)

    # TODO(Sanjay): We need a send without the extra copy.
    def send(self, data: bytearray) -> bool:
        """Send a single block of data by copying it.

        The data is typically a serialized flatbuffers buffer.

        Channels have a configured frequency. Messages cannot be sent faster than this
        value. The combined frequency of messages sent from all tasks is what matters,
        not the frequency that any single task sends at.

        Returns:
            bool: True if the data was sent successfully.
        """
        c_data = ffi.from_buffer(data)
        return self._c_sender.send(self._c_sender, c_data, len(data))


class Task:
    """A simple wrapper for a coroutine.

    It tracks whether or not the task needs to be resumed when the event loop runs.
    """

    def __init__(self, coro: Coroutine) -> None:
        self._coro: Coroutine = coro
        self._resume_on_run: bool = False

    def resume(self) -> None:
        """Resume the coroutine."""
        self._coro.send(None)


class EventLoopRuntime:
    """An async runtime on top of the C EventLoop interface.

    This runtime is AOS-specific, and incompatible with Python's asyncio and other
    generic async constructs. Therefore tasks that are spawned by this runtime can only
    await the objects provided by this module. Awaiting anything else is undefined
    behavior.

    The event loop runtime provides the interface for:
    * creating timers, fetchers, and senders
    * spawning tasks
    * auxiliary event loop functionality such as querying time or whether we're running

    Typically the runtime isn't directly created and managed by the user. Rather, it is
    provided by an event loop implementation or an event loop factory. For usage
    examples, check the relevant impementation's module. The runtime is designed to be
    used as a context manager.
    """

    def __init__(self, c_event_loop) -> None:
        self._c_event_loop = c_event_loop
        if self.is_running():
            raise RuntimeError(
                "Runtime must be created before the event loop is run.")
        self._timers: List[Timer] = []
        self._fetchers: Dict[Channel, Fetcher] = {}
        self._senders: Dict[Channel, Sender] = {}
        self._tasks: List[Task] = []
        self._current_task: Task = None

        self._handle = None
        self._on_run: Future = None

    def __enter__(self) -> "EventLoopRuntime":
        self.init()
        return self

    def __exit__(
        self,
        exc_type: Optional[Type[BaseException]],
        exc_value: Optional[BaseException],
        traceback: Optional[Any],
    ) -> bool:
        self.close()
        return False

    def init(self) -> None:
        """Register an on run callback with the event loop.

        This will resume all tasks that are awaiting `on_run` when the event loop runs.
        """
        self._handle = ffi.new_handle(self)
        self._c_event_loop.on_run(self._c_event_loop,
                                  EventLoopRuntime._on_run_callback,
                                  self._handle)
        self._on_run = Future()
        self._on_run.add_done_callback(self.__resume_on_run)

    def close(self) -> None:
        """Deallocate objects that were created using this runtime.

        This must happen before the event loop is destroyed.
        """
        for timer in self._timers:
            lib.destroy_timer_handler(timer._c_timer_handle)
        for _, fetcher in self._fetchers.items():
            lib.destroy_fetcher(fetcher._c_fetcher)
        for _, sender in self._senders.items():
            lib.destroy_sender(sender._c_sender)

    def is_running(self) -> bool:
        """Indicate if the event loop is running.

        Returns:
            bool: True if the event loop is running.
        """
        return self._c_event_loop.is_running(self._c_event_loop)

    def monotonic_now(self) -> int:
        """Read the current time on the monotonic clock.

        Returns:
            int: Nanoseconds since epoch on the monotonic clock.
        """
        return self._c_event_loop.monotonic_now(self._c_event_loop)

    def add_timer(self) -> Timer:
        """Create a timer.

        Returns:
            Timer: A Timer object. Its `tick` method can be awaited after it has been
                scheduled.
        """
        timer = Timer(self)
        self._timers.append(timer)
        return timer

    def make_fetcher(self, channel_name: str,
                     channel_type_cls: Type[Any]) -> Fetcher:
        """Create a fetcher (or fetch an existing one) for the specified channel.

        A fetcher for a given channel is shared among tasks. The state of the fetcher is
        also shared. This means if two tasks poll concurrently for a message, one of
        them could get (True, Message) and the other could get (False, Message).

        Returns:
            Fetcher: A Fetcher object. Its `next` method can be awaited to yield until
                the next message arrives on the channel. Its `fetch*` methods can be
                used to poll the channel for a message in a non-blocking way.
        """
        channel = Channel(channel_name, channel_type_cls)
        if channel in self._fetchers:
            fetcher = self._fetchers[channel]
        else:
            fetcher = Fetcher(self, channel)
            self._fetchers[channel] = fetcher
        fetcher.register_task(self._get_current_task())
        return fetcher

    def make_sender(self, channel_name: str,
                    channel_type_cls: Type[Any]) -> Sender:
        """Create a sender (or fetch an existing one) for the specified channel.

        Returns:
            Sender: A Sender object. Its `send` method can be used to send a serialized
                flatbuffers data buffer.
        """
        channel = Channel(channel_name, channel_type_cls)
        if channel in self._senders:
            return self._senders[channel]
        sender = Sender(self, channel)
        self._senders[channel] = sender
        return sender

    def spawn_task(self, coro: Coroutine) -> None:
        """Spawn a single task.

        The task will run till its first yield point (typically an "await").
        """
        if not asyncio.iscoroutine(coro):
            raise TypeError()
        task = Task(coro)
        self._tasks.append(task)
        self.__start(task)

    def spawn_tasks(self, tasks: List[Coroutine]) -> None:
        """Spawn all the tasks in the provided list."""
        for task in tasks:
            self.spawn_task(task)

    async def on_run(self) -> None:
        """An awaitable function that will resume the task when the event loop runs."""
        if not self._on_run:
            raise RuntimeError("Use EventLoopRuntime as a context manager")
        if self.is_running():
            raise RuntimeError()
        self._get_current_task()._resume_on_run = True
        await self._on_run

    def _set_current_task(self, task: Task) -> None:
        self._current_task = task

    def _get_current_task(self) -> Task:
        return self._current_task

    def _loop(self):
        return self._c_event_loop

    def __start(self, task: Task) -> None:
        try:
            self._set_current_task(task)
            task.resume()
        except StopIteration:
            pass
        finally:
            self._set_current_task(None)

    def __resume_on_run(self) -> None:
        if not self._on_run.done():
            raise FutureResultNotReadyError()
        for task in self._tasks:
            if task._resume_on_run:
                try:
                    self._set_current_task(task)
                    task.resume()
                except StopIteration:
                    pass
                finally:
                    self._set_current_task(None)

    @staticmethod
    @ffi.callback("void(void *)")
    def _on_run_callback(runtime_handle) -> None:
        runtime = ffi.from_handle(runtime_handle)
        runtime._on_run.set_result()
