import sys
import io
import contextlib

from absl.testing import absltest
import flatbuffers
import numpy

from aos.events import util
from aos.events.simulated_event_loop import SimulatedEventLoopFactory
from aos.events.simulated_event_loop import ffi
from aos.events.event_loop import MessagesSentTooFastError
from test_message_fbs_py.aos.TestMessage import TestMessageT

VECTOR_VALUE = numpy.array([1, 8, 6, 8], dtype='int32')


class EventLoopTest(absltest.TestCase):

    def setUp(self):
        self._config = util.ConfigurationBuffer(
            util.locate("aos/aos/events/event_loop_py_config.bfbs"))
        with contextlib.ExitStack() as stack:
            self._factory = stack.enter_context(
                SimulatedEventLoopFactory(self._config))
            self.addCleanup(stack.pop_all().close)

    def test_basics(self):
        """Tests the basic happy paths for the common APIs."""
        send_loop = self._factory.make_event_loop("primary", "")
        assert send_loop.name() == "primary"
        receive_loop1 = self._factory.make_event_loop("loop1", "")
        receive_loop2 = self._factory.make_event_loop("loop2", "")

        fetcher = receive_loop2.make_fetcher(TestMessageT, "/test")
        watcher_count = 0
        timer_count = 0

        def handle_test_message(message):
            nonlocal watcher_count, fetcher, send_loop
            assert numpy.array_equal(message.vector, VECTOR_VALUE)
            watcher_count += 1

            fetched = fetcher.fetch()
            assert fetched is not None
            assert numpy.array_equal(fetched.vector, VECTOR_VALUE)

            assert send_loop.realtime_now_ns() > 0

        def handle_timer():
            nonlocal timer_count
            timer_count += 1

        sender = send_loop.make_sender(TestMessageT, "/test")
        receive_loop1.make_watcher(TestMessageT, "/test", handle_test_message)
        timer = send_loop.add_timer(handle_timer)
        receive_loop1.set_runtime_affinity([1])

        def on_run():
            nonlocal send_loop
            assert send_loop.is_running()
            message = TestMessageT()
            message.vector = VECTOR_VALUE
            sender.send(message)

            timer.schedule_ns(send_loop.monotonic_now_ns(), 1_000_000)

        send_loop.on_run(on_run)

        assert fetcher.fetch() is None
        assert fetcher.fetch_next() is None
        assert not send_loop.is_running()

        self._factory.run_for_ns(10_500_000)

        self.assertEqual(watcher_count, 1)
        self.assertEqual(timer_count, 11)

    def test_send_too_fast(self):
        """Tests the Python exception raised when sending too fast."""
        send_loop = self._factory.make_event_loop("primary", "")
        sender = send_loop.make_sender(TestMessageT, "/test")

        def send_too_fast():
            for _ in range(1000):
                sender.send(TestMessageT())

        send_loop.on_run(send_too_fast)
        with self.assertRaises(MessagesSentTooFastError):
            self._factory.run_for_ns(1)

    def test_reraise_exception(self):
        """Tests re-raising a custom Python exception."""

        class TestError(Exception):
            pass

        def raise_exception():
            raise TestError()

        send_loop = self._factory.make_event_loop("primary", "")
        send_loop.on_run(raise_exception)
        with self.assertRaises(TestError):
            self._factory.run_for_ns(1)

    def test_dont_reraise_exception(self):
        """Tests that a custom Python exception is printed but not re-raised
        when event processing stops for a different reason."""

        class TestError(Exception):
            pass

        exit_handle = self._factory.make_exit_handle()

        def raise_exception():
            exit_handle.exit()
            raise TestError()

        send_loop = self._factory.make_event_loop("primary", "")
        send_loop.on_run(raise_exception)

        captured_stderr = io.StringIO()
        with contextlib.redirect_stderr(captured_stderr):
            self._factory.run_for_ns(1)
        stderr_value = captured_stderr.getvalue()
        self.assertIn('processing stopped for a different reason',
                      stderr_value)
        self.assertIn('TestError', stderr_value)

    def test_outgrow_builder_in_place(self):
        send_loop = self._factory.make_event_loop("primary", "")
        sender = send_loop.make_sender(TestMessageT, "/test")
        message = TestMessageT()
        message.vector = [0] * 1000
        with self.assertRaisesRegex(
                flatbuffers.builder.BuilderSizeError,
                'Failed to allocate for { "name": "/test", "type": "TestMessageT" } with max_size of 1000'
        ):
            sender.send(message)

    def test_retain_fetcher_buffer(self):
        """Tests retaining the entire buffer from a fetcher."""
        send_loop = self._factory.make_event_loop("primary", "")
        sender = send_loop.make_sender(TestMessageT, "/test")

        receive_loop = self._factory.make_event_loop("loop1", "")
        fetcher = receive_loop.make_fetcher(TestMessageT, "/test")

        sender.send(TestMessageT())
        assert fetcher.do_fetch()

        context = fetcher.raw_context()
        retained_buffer = context.buffer
        assert retained_buffer is not None
        assert retained_buffer[0] is not None
        with self.assertRaises(BufferError):
            fetcher.do_fetch()

    def test_retain_fetcher_buffer_view(self):
        """Tests retaining a view of the buffer from a fetcher."""
        send_loop = self._factory.make_event_loop("primary", "")
        sender = send_loop.make_sender(TestMessageT, "/test")

        receive_loop = self._factory.make_event_loop("loop1", "")
        fetcher = receive_loop.make_fetcher(TestMessageT, "/test")

        sender.send(TestMessageT())
        assert fetcher.do_fetch()

        context = fetcher.raw_context()
        retained_buffer_view = context.buffer[:]
        assert retained_buffer_view[0] is not None
        with self.assertRaises(BufferError):
            fetcher.do_fetch()

    def test_retain_watcher_buffer(self):
        send_loop = self._factory.make_event_loop("primary", "")
        sender = send_loop.make_sender(TestMessageT, "/test")

        retained_buffer = None

        def handle_test_message(context):
            nonlocal retained_buffer
            retained_buffer = context.buffer

        receive_loop = self._factory.make_event_loop("loop1", "")
        receive_loop.make_context_watcher(TestMessageT, "/test",
                                          handle_test_message)

        send_loop.on_run(lambda: sender.send(TestMessageT()))

        with self.assertRaises(BufferError):
            self._factory.run_for_ns(500_000)

    def test_strings_arrays(self):
        """Tests messages with strings and array. When numpy is importable,
        the FlatBuffers Python object based API retains references to the buffer
        by default."""
        send_loop = self._factory.make_event_loop("primary", "")
        sender = send_loop.make_sender(TestMessageT, "/test")

        retained_message = None

        def handle_test_message(message):
            nonlocal retained_message
            retained_message = message

        receive_loop1 = self._factory.make_event_loop("loop1", "")
        receive_loop1.make_watcher(TestMessageT, "/test", handle_test_message)
        receive_loop2 = self._factory.make_event_loop("loop2", "")
        fetcher = receive_loop2.make_fetcher(TestMessageT, "/test")

        send_loop.on_run(
            lambda: sender.send(TestMessageT(vector=[1, 2, 3], string='abc')))

        self._factory.run_for_ns(500_000)
        assert fetcher.fetch() is not None
        assert isinstance(retained_message.vector, numpy.ndarray)

    def test_plain_run(self):
        send_loop = self._factory.make_event_loop("primary", "")
        send_loop.on_run(lambda: self._factory.make_exit_handle().exit())
        self._factory.run()


if __name__ == "__main__":
    util.init(sys.argv)
    absltest.main()
