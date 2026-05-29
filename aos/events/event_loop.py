from typing import Any, List, Type, Callable, Optional
import traceback
import sys
import enum

import flatbuffers
from absl import logging

from aos.events.event_loop_c import ffi, lib
from aos.configuration_fbs_py.aos.Configuration import Configuration, Node
from aos.events.util import ConfigurationBuffer
'''
A friendly, callback-based Python interface to EventLoop.

# Overview

This EventLoop interface prioritizes ease of use and object ownership over full
control, because Python does not offer fine-grained object lifetime control.
This also means that memory may be allocated and freed from Python during
realtime operation, which is hard to prevent Python from doing anyways.
See the [Design details] section for details.

A direct mapping of the C++ API is preferred to more Pythonic names and
argument conventions.

We expose the flatbuffers object based API by default, which provides an
interface straightforward to use in Python without concerns around memory
lifetime and ownership. This is another source of memory allocations.

# Usage

Create an `EventLoop` and then enter it as a context manager. Within this
context, all interaction with the `EventLoop` should happen. All user-facing
objects are tied to this context, and will become inactive when the context is
exited. Many programs will be entirely contained within this context; the main
use case for multiple of them is unit testing.

After an object is inactive, it should not be interacted with. Any interactions
may do any of the following:
    * Normal behavior
    * Nothing
    * Raise `ReferenceError`
Note that no user-provided callbacks will be executed after the context is
exited, which is the primary source of interacting with these objects.

# Design details

Python's language and patterns are both designed to abstract away object
ownership. It does provide context managers to explicitly manage ownership of
resources, but these contexts can only cover lexical scopes, and attempting
to store them in variables (eg `contextlib.ExitStack`) is awkward and still has
to tie back to a lexical scope. To resolve this, we enforce a lexical scope
around everything, and tie all the C++ object lifetimes to that. This also
comes with the benefit of not needing to track cross-language reference counts.
(Try/finally is an alternative, but it is even less ergonomic and still limited
to lexical scopes.)

`EventLoop` itself maintains ownership of all C++ objects, and ensures they are
all destroyed in its `__exit__`.
This means that any direct or indirect Python references will remain valid for
the duration, and any remaining invalidated ones can be safely deactivated so
they raise exceptions instead of segfaulting.
This also allows `EventLoop` to ensure no user callbacks are called after
`__exit__`, which is the main source of potential use-after-free in this style.

We expose the underlying C++ memory directly. This is needed for performant
access to large messages, but it is difficult to use correctly from Python. We
wrap all of this access in further context managers which verify a lack of
Python references at `__exit__` time.

## Flatbuffers

The Python flatbuffers API (the non-object based one) solves this problem by
storing a Python-owned buffer in all of the Python objects. This makes these
objects simpler than the design for `EventLoop`'s child objects, and makes more
sense here.

Given a known Python-owned buffer, it is simple to convert between the offsets
tracked in Python and a pointer passed through C++ code. The Python
`FlatbufferDetachedBuffer` implements these conversions, which we use there.
This means we can freely expose these objects to Python callers and rely on
Python refcounting to manage the lifetime of the backing buffer.

However, this means the Python buffer must be known, which is not easy to pass
through a C++ API in a robust way. We require a Python buffer at `EventLoop`
construction time, and any caller with a C++-owned (or generically-owned)
buffer must copy it.
'''


class SchedulingPolicy(enum.Enum):
    OTHER = lib.aos_const_scheduling_policy_other()
    FIFO = lib.aos_const_scheduling_policy_fifo()
    RR = lib.aos_const_scheduling_policy_rr()


class InternalProxy:
    """Wraps an object which can be cleared later.

    See the module-level discussion for how this is used."""

    def __init__(self, target):
        self._event_loop_proxy_target = target

    def __getattr__(self, name):
        if self._event_loop_proxy_target is None:
            raise ReferenceError('Target has been cleared')
        return getattr(self._event_loop_proxy_target, name)


class PythonChannel:
    """A Python wrapper around the concept of an AOS channel.

    This bundles the name and type together, and provides some helpers related
    to them.

    Note that this is not a wrapper for the `aos.Channel` table."""

    def __init__(self, channel_type: Type[Any], channel_name: str) -> None:
        assert channel_type.__name__.endswith(
            'T'), "Must use the flatbuffers object type"

        self._channel_type = channel_type
        self._channel_name = channel_name

    def c_name(self):
        return ffi.new("char[]", self._channel_name.encode('utf-8'))

    @property
    def channel_name(self):
        return self._channel_type

    def c_type(self):
        return ffi.new("char[]", self.fbs_module().encode('utf-8'))

    def fbs_module(self):
        """Returns the fbs module name. This is how the C API identifies this type."""
        module = self._channel_type.__module__
        return module.split('_fbs_py.', 1)[1]

    def object_from_buffer(self, buffer):
        return self._channel_type.InitFromPackedBuf(buffer, 0)

    def stripped_channel_to_string(self):
        """Re-implementation `aos::StrippedChannelToString` with the information in this type."""
        return '{ "name": "%s", "type": "%s" }' % (self._channel_name,
                                                   self._channel_type.__name__)

    def assert_message_type(self, message_t):
        assert type(message_t) == self._channel_type


class ContextView:
    """A Python view of a C++ `const Context&`.

    All of the public fields other than `channel_wrapper` correspond one-to-one
    with fields in aos::Context, see aos/events/context.h for detailed documentation.
    """

    def __init__(self, c_context, channel_wrapper: PythonChannel) -> None:
        """Caller is responsible for ensuring `c_context.data` remains valid for
        the lifespan of the created buffer and any views of it."""
        self.monotonic_event_time_ns = c_context.monotonic_event_time
        self.realtime_event_time_ns = c_context.realtime_event_time
        self.queue_index = c_context.queue_index
        self.remote_queue_index = c_context.remote_queue_index
        self.size = c_context.size
        self._data = c_context.data

        self.channel_wrapper = channel_wrapper
        if c_context.size:
            # This one is a bit tricky. We cannot wrap this object here, but we
            # must wrap it elsewhere.
            #
            # `ffi.buffer` is a very incomplete implementation of the buffer
            # protocol, and if we expose it directly we would open up a lot of
            # use-after-frees. However, once we wrap it in a memoryview, we lose
            # the ability to track references because memoryview-to-memoryview
            # references (slicing, constructor, etc) have an optimization so all
            # derived memoryview objects refer to the same internal object with
            # its own refcount that is inaccessible from outside (via
            # sys.getrefcount, release(), or any other way).
            #
            # Our solution is to wrap this object in a new memoryview every
            # time it is accessed, so each of those memoryview objects will
            # create a new memoryview-internal object to retain a refcount on
            # this object. This means that any further memoryview objects will
            # always retain a refcount somewhere we can see it.
            #
            # This problem could also be solved by finding or writing a Python
            # buffer type which retains references to the source buffer (instead
            # of copying the data) and doesn't interoperate with memoryview's
            # internal tracking. I could not find one of those, and writing it
            # would be more work than the chosen approach.
            self._buffer = ffi.buffer(ffi.cast("const char *", c_context.data),
                                      c_context.size)
        else:
            self._buffer = None

    @property
    def buffer(self):
        # See comment in __init__ about why wrapping this in a fresh memoryview
        # every time is important.
        if self._buffer:
            return memoryview(self._buffer)

    def copy_message_object(self) -> Any:
        """Returns a copy (owned by Python) of the message in the context.

        Returns None if there is no message in the context."""
        if self.buffer:
            return self.channel_wrapper.object_from_buffer(self.buffer)

    def release_buffer(self):
        """Releases the buffer, and raises BufferError if any references have been retained.

        This is important to ensure that no Python references are retained to a
        freed/reused C++-owned buffer.

        Note that somebody could catch the BufferError and still have a reference to a freed
        buffer. That's not an immediate problem, which enables debuggers/etc to function.
        They could still cause a use-after-free by attempting to access the freed buffer, but
        that's inevitable with Python <-> C interop at this level, so we accept the risk."""
        if self._buffer is None:
            return
        refcount = sys.getrefcount(self._buffer)
        assert refcount >= 2, "Should always have refs from member variable and temporary"
        if refcount > 2:
            raise BufferError(
                "Excess references to an AOS context buffer have been retained"
            )
        self._buffer = None

    def __enter__(self):
        return self

    def __exit__(
        self,
        exc_type: Optional[Type[BaseException]],
        exc_value: Optional[BaseException],
        tb: Optional[Any],
    ) -> bool:
        self.release_buffer()
        return False


class BuilderInPlace(flatbuffers.Builder):
    """Allows building a flatbuffer table in place from Python.

    This currently involves digging into flatbuffers internals, and may break in
    the future. Ideally we would upstream this."""

    def __init__(self, buffer, channel_wrapper: PythonChannel):
        # This allocates a 1-byte buffer which we will never use. Not ideal, but not a big problem.
        super().__init__(1)
        self._channel_wrapper = channel_wrapper
        self.Bytes = buffer
        self.head = len(self.Bytes)

    def GrowByteBuffer(self):
        raise flatbuffers.builder.BuilderSizeError(
            "Failed to allocate for %s with max_size of %d" %
            (self._channel_wrapper.stripped_channel_to_string(), len(
                self.Bytes)))


def _intercept_exception(
    exc_type: Type[BaseException],
    exc_value: BaseException,
    traceback: Any,
):
    global _global_exception_list, _global_exit_handle
    if _global_exception_list is None:
        # TODO(Brian): Hook up enough of a C++ -> Python example to run this and make sure the traceback is printed.
        logging.fatal(
            'Python exception encountered without Python context to re-raise to',
            exc_info=exc_value)
    _global_exception_list.append((exc_type, exc_value, traceback))
    _global_exit_handle.exit_with_python_exception()


class Timer:
    """Wraps an `aos::TimerHandler`."""

    def __init__(self, c_event_loop, callback: Callable[[], None]) -> None:
        """No methods may be called after `c_event_loop` is destroyed."""
        self._handle = ffi.new_handle(self)
        self._c_timer = lib.aos_event_loop_add_timer(c_event_loop,
                                                     Timer._timer_callback,
                                                     self._handle)
        self._callback = callback

    def schedule_ns(self, start_ns: int, period_ns: int = 0) -> None:
        lib.aos_timer_handler_schedule(self._c_timer, start_ns, period_ns)

    def disable(self) -> None:
        lib.aos_timer_handler_disable(self._c_timer)

    def set_name(self, name: str) -> None:
        name_bytes = name.encode('utf-8')
        lib.aos_timer_set_name(self._c_timer, ffi.from_buffer(name_bytes),
                               len(name_bytes))

    def name(self) -> str:
        name_data = ffi.new('const char **')
        name_size = ffi.new('size_t *')
        lib.aos_timer_get_name(self._c_timer, name_data, name_size)
        return ffi.unpack(name_data[0], name_size[0]).decode('utf-8')

    @ffi.callback("void(void *)", onerror=_intercept_exception)
    def _timer_callback(timer_handle) -> None:
        with ExceptionPassthroughInterceptor():
            timer = ffi.from_handle(timer_handle)
            timer._callback()


class Fetcher:
    """Wraps an `aos::Fetcher`."""

    def __init__(self, c_event_loop, channel_wrapper: PythonChannel):
        """Caller takes ownership of the created C resources."""
        self._channel_wrapper = channel_wrapper
        c_channel_name = channel_wrapper.c_name()
        c_channel_type = channel_wrapper.c_type()
        self._c_fetcher = lib.aos_event_loop_make_fetcher(
            c_event_loop, c_channel_name, c_channel_type)

        self._context: Optional[ContextView] = None

    def do_fetch_next(self) -> bool:
        """Fetches the next message in the queue without blocking. Returns true
        if there was a new message."""
        self.release_context_buffer()
        return lib.aos_fetcher_fetch_next(self._c_fetcher)

    def fetch_next(self) -> Any:
        """Fetches the next message in the queue without blocking. If there was
        a new message, returns a copy (owned by Python) of it.

        Note that the previous message in the context will remain valid if this
        returns None."""
        if self.do_fetch_next():
            return self.raw_context().copy_message_object()

    def do_fetch(self) -> bool:
        """Fetches the latest message without blocking. Returns true if there
        was any message."""
        self.release_context_buffer()
        return lib.aos_fetcher_fetch(self._c_fetcher)

    def fetch(self) -> Any:
        """Fetches the latest message without blocking. If there was any
        message, returns a copy (owned by Python) of it."""
        if self.do_fetch():
            return self.raw_context().copy_message_object()

    def raw_context(self) -> ContextView:
        """Returns a view of the current context.

        The buffer of the returned `ContextView` must have no active views
        when any further methods are called on this object (including calling
        this method again or exiting the context of the parent `EventLoop`). It
        is strongly recommended to use the context manager protocol on any views
        of the buffer to ensure this.

        Note that the members other than the buffer are read-only copies, and
        the returned Python object will not update on further calls to `fetch`
        and friends."""
        self.release_context_buffer()
        c_context = lib.aos_fetcher_context(self._c_fetcher)
        context = ContextView(c_context, self._channel_wrapper)
        self._context = context
        return context

    def release_context_buffer(self):
        if self._context is None:
            return
        self._context.release_buffer()
        self._context = None


class SenderError(Exception):
    pass


class MessagesSentTooFastError(SenderError):
    pass


class InvalidRedzoneError(SenderError):
    pass


class Sender:
    """Wraps an `aos::Sender`."""

    def __init__(self, c_event_loop, channel_wrapper: PythonChannel):
        """Caller takes ownership of the created C resources."""
        self._channel_wrapper = channel_wrapper
        c_channel_name = channel_wrapper.c_name()
        c_channel_type = channel_wrapper.c_type()
        self._c_sender = lib.aos_event_loop_make_sender(
            c_event_loop, c_channel_name, c_channel_type)

    def send(self, message_t: Any) -> None:
        self._channel_wrapper.assert_message_type(message_t)
        builder = BuilderInPlace(self.raw_buffer(), self._channel_wrapper)
        root = message_t.Pack(builder)
        builder.Finish(root)
        result = lib.aos_sender_send(self._c_sender,
                                     len(builder.Bytes) - builder.Head())
        if result == lib.aos_const_raw_sender_error_ok():
            return
        if result == lib.aos_const_raw_sender_error_messages_sent_too_fast():
            raise MessagesSentTooFastError()
        if result == lib.aos_const_raw_sender_error_invalid_redzone():
            raise InvalidRedzoneError()
        raise RuntimeError('Unknown aos_sender_send result: %d' % result)

    def raw_buffer(self):
        """Returns a reference to the raw buffer. This is owned by C++ and may
        be invalidated at many points. For advanced usage only."""
        return ffi.buffer(lib.aos_sender_data(self._c_sender),
                          lib.aos_sender_size(self._c_sender))


_global_exception_list = None
_global_exit_handle = None


class ExceptionPassthroughReceiver:
    """Receives exceptions which are saved while processing events.

    Any saved exceptions are re-thrown from `reraise_exceptions`. Any saved
    exceptions which are not re-thrown will be logged at context exit."""

    def __init__(self, exit_handle):
        self._previous_list = None
        self._previous_exit_handle = None
        self._exit_handle = exit_handle

    def __enter__(self):
        global _global_exception_list, _global_exit_handle
        assert self._previous_list is None
        assert self._previous_exit_handle is None
        self._previous_list = _global_exception_list
        self._previous_exit_handle = _global_exit_handle
        _global_exception_list = list()
        _global_exit_handle = self._exit_handle
        return self

    def reraise_exceptions(self) -> None:
        global _global_exception_list
        assert _global_exception_list is not None
        exception_list = _global_exception_list
        _global_exception_list = list()

        if exception_list:
            # TODO(Brian): Remove everything else in favor of just this once we require Python 3.11+:
            #   raise BaseExceptionGroup('while processing events', exception_list)
            to_print = exception_list[1:]
            for i, exc_info in enumerate(to_print):
                logging.error(
                    'Additional exception %d/%d (first exception will be re-raised):'
                    % (i + 1, len(to_print)),
                    exc_info=exc_info)
            raise exception_list[0][1]

    def maybe_reraise_exceptions(self, error) -> None:
        if error is None:
            return
        if error.code == lib.aos_const_error_status_code_python_exception():
            self.reraise_exceptions()

    def __exit__(
        self,
        exc_type: Optional[Type[BaseException]],
        exc_value: Optional[BaseException],
        tb: Optional[Any],
    ) -> bool:
        global _global_exception_list, _global_exit_handle
        assert _global_exception_list is not None
        assert _global_exit_handle is not None
        exception_list = _global_exception_list
        _global_exception_list = self._previous_list
        self._previous_list = None
        _global_exit_handle = self._previous_exit_handle
        self._previous_exit_handle = None

        for i, exc_info in enumerate(exception_list):
            logging.error(
                'Exception %d/%d while processing events (processing stopped for a different reason):'
                % (i + 1, len(exception_list)),
                exc_info=exc_info)

        return False


class ExceptionPassthroughInterceptor:
    """Saves exceptions encountered while processing events."""

    def __enter__(self) -> None:
        pass

    def __exit__(
        self,
        exc_type: Optional[Type[BaseException]],
        exc_value: Optional[BaseException],
        traceback: Optional[Any],
    ) -> bool:
        if exc_value is not None:
            _intercept_exception(exc_type, exc_value, traceback)
        return True


class ExitHandle:
    """Wraps an `aos::ExitHandle`."""

    def __init__(self, c_exit_handle):
        """`close()` must be called on the result before `c_exit_handle` is destroyed."""
        self._c_exit_handle = c_exit_handle

    def close(self):
        lib.aos_exit_handle_destroy(self._c_exit_handle)

    def exit(self):
        lib.aos_exit_handle_exit(self._c_exit_handle)

    def exit_with_python_exception(self):
        lib.aos_exit_handle_exit_with_python_exception(self._c_exit_handle)


class Error:
    """Wraps a non-ok `aos::Status`. Handles memory management internally.

    The constructor is overriden to return `None` when passed `ffi.NULL`."""

    def __new__(cls, c_error):
        if c_error == ffi.NULL:
            return None
        return super().__new__(cls)

    def __init__(self, c_error):
        self._c_error = c_error

    @property
    def code(self):
        return lib.aos_error_code(self._c_error)

    def __del__(self):
        lib.aos_error_destroy(self._c_error)


class EventLoop:
    """Wraps an `aos::EventLoop`."""

    def __init__(self, c_event_loop: ffi.CData,
                 config_buffer: ConfigurationBuffer) -> None:
        """`close()` must be called on the result before `c_event_loop` is destroyed.

        `c_event_loop`'s configuration must lie within `config_buffer`. If you
        need to create an instance with a C++-owned `aos::Configuration`,
        create a copy into a Python-owned `ConfigurationBuffer` before calling
        this constructor."""
        self._c_event_loop = c_event_loop
        self._config_buffer = config_buffer
        self._proxies: List[InternalProxy] = list()
        self._timers: List[Timer] = list()
        self._fetchers: List[Fetcher] = list()
        self._senders: List[Sender] = list()
        self._callback_handles: List[ffi.Handle] = list()
        self._handle = ffi.new_handle(self)

        # Call this now to confirm our caller passed the correct `ConfigurationBuffer`.
        self.configuration()

    def close(self):
        for proxy in self._proxies:
            proxy._event_loop_proxy_target = None
        self._proxies.clear()

        self._timers.clear()
        for fetcher in self._fetchers:
            fetcher.release_context_buffer()
            lib.aos_fetcher_destroy(fetcher._c_fetcher)
        self._fetchers.clear()
        for sender in self._senders:
            lib.aos_sender_destroy(sender._c_sender)
        self._senders.clear()
        self._callback_handles.clear()

    def is_running(self) -> bool:
        """Indicate if the event loop is running.

        Returns:
            bool: True if the event loop is running.
        """
        return lib.aos_event_loop_is_running(self._c_event_loop)

    def monotonic_now_ns(self) -> int:
        """Read the current time on the monotonic clock.

        Returns:
            int: Nanoseconds since epoch on the monotonic clock.
        """
        return lib.aos_event_loop_monotonic_now(self._c_event_loop)

    def realtime_now_ns(self) -> int:
        """Read the current time on the realtime clock.

        Returns:
            int: Nanoseconds since epoch on the realtime clock.
        """
        return lib.aos_event_loop_realtime_now(self._c_event_loop)

    def name(self) -> str:
        name_data = ffi.new('const char **')
        name_size = ffi.new('size_t *')
        lib.aos_event_loop_get_name(self._c_event_loop, name_data, name_size)
        return ffi.unpack(name_data[0], name_size[0]).decode('utf-8')

    def _add_proxy(self, target):
        proxy = InternalProxy(target)
        self._proxies.append(proxy)
        return proxy

    def add_timer(self, callback: Callable[[], None]) -> Timer:
        """Create a timer.

        Args:
            callback: Function which will be called each time the timer expires.

        Returns:
            Timer: A Timer object.
        """
        timer = Timer(self._c_event_loop, callback)
        self._timers.append(timer)
        return self._add_proxy(timer)

    def make_fetcher(self, channel_type: Type[Any],
                     channel_name: str) -> Fetcher:
        """Create a fetcher for the specified channel.

        Returns:
            Fetcher: A Fetcher object.
        """
        fetcher = Fetcher(self._c_event_loop,
                          PythonChannel(channel_type, channel_name))
        self._fetchers.append(fetcher)
        return self._add_proxy(fetcher)

    def make_sender(self, channel_type: Type[Any],
                    channel_name: str) -> Sender:
        """Create a sender for the specified channel.

        Returns:
            Sender: A Sender object.
        """
        sender = Sender(self._c_event_loop,
                        PythonChannel(channel_type, channel_name))
        self._senders.append(sender)
        return self._add_proxy(sender)

    def make_watcher(self, channel_type: Type[Any], channel_name: str,
                     callback: Callable[[Any], None]) -> None:
        """Create a watcher which only receives the message for the specified channel.

        For advanced usage accessing the message in other ways, see `make_context_watcher`."""
        self.make_context_watcher(
            channel_type, channel_name,
            lambda context: callback(context.copy_message_object()))

    def make_context_watcher(self, channel_type: Type[Any], channel_name: str,
                             callback: Callable[[ContextView], None]) -> None:
        """Create a watcher which receives the context for the specified channel.

        The buffer of the returned `ContextView` must have no active views when
        the callback returns. It is strongly recommended to use the context
        manager protocol on any views of the buffer to ensure this.
        """
        channel_wrapper = PythonChannel(channel_type, channel_name)

        def wrapped_callback(c_context):
            nonlocal callback, channel_wrapper

            with ContextView(c_context, channel_wrapper) as context:
                callback(context)

        c_channel_name = channel_wrapper.c_name()
        c_channel_type = channel_wrapper.c_type()
        callback_handle = ffi.new_handle(wrapped_callback)
        self._callback_handles.append(callback_handle)
        lib.aos_event_loop_make_watcher(self._c_event_loop, c_channel_name,
                                        c_channel_type,
                                        EventLoop._watcher_context_callback,
                                        callback_handle)

    @ffi.callback("void(const aos_context_t *, const void *, void *)",
                  onerror=_intercept_exception)
    def _watcher_context_callback(c_context, c_message, callback_handle):
        with ExceptionPassthroughInterceptor():
            callback = ffi.from_handle(callback_handle)
            callback(c_context)

    def on_run(self, callback: Callable) -> None:
        callback_handle = ffi.new_handle(callback)
        self._callback_handles.append(callback_handle)
        lib.aos_event_loop_on_run(self._c_event_loop,
                                  EventLoop._on_run_callback, callback_handle)

    @ffi.callback("void(void *)", onerror=_intercept_exception)
    def _on_run_callback(callback_handle):
        with ExceptionPassthroughInterceptor():
            callback = ffi.from_handle(callback_handle)
            callback()

    def set_runtime_realtime_priority(
            self,
            priority: int,
            scheduling_policy: SchedulingPolicy = SchedulingPolicy.FIFO
    ) -> None:
        lib.aos_event_loop_set_runtime_realtime_priority(
            self._c_event_loop, priority, scheduling_policy.value,
            lib.aos_const_realtime_policy_no_mode())

    def set_runtime_affinity(self, affinity: [int]) -> None:
        affinity_c = ffi.new('int32_t[]', affinity)
        lib.aos_event_loop_set_runtime_affinity(self._c_event_loop, affinity_c,
                                                len(affinity))

    def node(self) -> Node:
        return self._config_buffer.c_to_fbs(
            Node, lib.aos_event_loop_node(self._c_event_loop))

    def configuration(self) -> Configuration:
        return self._config_buffer.c_to_fbs(
            Configuration,
            lib.aos_event_loop_configuration(self._c_event_loop))

    @property
    def config_buffer(self) -> ConfigurationBuffer:
        return self._config_buffer
