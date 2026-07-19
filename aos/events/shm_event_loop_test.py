import sys
import os
import contextlib

from absl.testing import absltest
from absl import flags

flags.DEFINE_string("shm_base", None, "Shared memory base directory")

from aos.events import util
from aos.events.shm_event_loop import ShmEventLoop, READABLE_EVENT, WRITABLE_EVENT
from aos.events.event_loop_fbs_py.aos.timing.Report import ReportT


class ShmEventLoopTest(absltest.TestCase):

    def test_set_name(self):
        with ShmEventLoop(
                util.ConfigurationBuffer(
                    util.locate("aos/aos/events/event_loop_py_config.bfbs"))
        ) as event_loop:
            # The default name is the program's: python.exe on Windows.
            self.assertEqual(
                event_loop.name(),
                "python" if sys.platform == "win32" else "python3")
            event_loop.set_name("new_name")
            self.assertEqual(event_loop.name(), "new_name")

    def test_on_fd_events(self):
        with contextlib.ExitStack() as exit_stack:
            if sys.platform == "win32":
                # Aio watches sockets on Windows, not the C runtime's file
                # descriptors, so a socket pair stands in for the pipe.  Its
                # fileno() is the SOCKET the event loop takes.
                import socket
                read_sock, write_sock = socket.socketpair()
                for sock in (read_sock, write_sock):
                    sock.setblocking(False)
                    exit_stack.callback(sock.close)
                read_fd, write_fd = read_sock.fileno(), write_sock.fileno()
                write = write_sock.send
            else:
                if hasattr(os, 'pipe2'):
                    read_fd, write_fd = os.pipe2(os.O_NONBLOCK)
                else:
                    import fcntl
                    read_fd, write_fd = os.pipe()
                    for fd in (read_fd, write_fd):
                        flags = fcntl.fcntl(fd, fcntl.F_GETFL)
                        fcntl.fcntl(fd, fcntl.F_SETFL, flags | os.O_NONBLOCK)
                exit_stack.callback(os.close, read_fd)
                exit_stack.callback(os.close, write_fd)
                write = lambda data: os.write(write_fd, data)

            event_loop = exit_stack.enter_context(
                ShmEventLoop(
                    util.ConfigurationBuffer(
                        util.locate(
                            "aos/aos/events/event_loop_py_config.bfbs"))))
            write_call_count = 0
            read_call_count = 0

            def write_callback(events):
                nonlocal write_call_count, read_call_count, event_loop
                event_loop.set_fd_events(write_fd, 0)
                assert events & WRITABLE_EVENT
                write_call_count += 1
                assert read_call_count == 0
                write(b'\0' * 100)

            def read_callback(events):
                nonlocal write_call_count, read_call_count, event_loop
                event_loop.set_fd_events(read_fd, 0)
                assert events & READABLE_EVENT
                read_call_count += 1
                assert write_call_count == 1
                event_loop.make_exit_handle().exit()

            event_loop.on_fd_events(write_fd, write_callback)
            exit_stack.callback(event_loop.delete_fd, write_fd)
            event_loop.set_fd_events(write_fd, WRITABLE_EVENT)
            event_loop.on_fd_events(read_fd, read_callback)
            exit_stack.callback(event_loop.delete_fd, read_fd)
            event_loop.set_fd_events(read_fd, READABLE_EVENT)

            event_loop.run()
            self.assertEqual(write_call_count, 1)
            self.assertEqual(read_call_count, 1)

    def test_basics(self):
        """Tests the basic happy paths for the common APIs."""
        with ShmEventLoop(
                util.ConfigurationBuffer(
                    util.locate("aos/aos/events/event_loop_py_config.bfbs"))
        ) as event_loop:
            event_loop.lock_to_thread()

    def test_skip_timing_report(self):
        with ShmEventLoop(
                util.ConfigurationBuffer(
                    util.locate("aos/aos/events/event_loop_py_config.bfbs"))
        ) as event_loop:
            event_loop.skip_timing_report()
            event_loop.skip_aos_log()
            event_loop.make_watcher(ReportT, "/aos", lambda m: None)
            exit_handle = event_loop.make_exit_handle()
            event_loop.on_run(lambda: exit_handle.exit())
            event_loop.run()


if __name__ == "__main__":
    shm_base = os.environ.get("TEST_TMPDIR", "/tmp") + "/aos"
    sys.argv.append(f"--shm_base={shm_base}")
    util.init(sys.argv)
    absltest.main()
