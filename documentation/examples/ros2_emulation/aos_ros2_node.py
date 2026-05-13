from documentation.examples.ros2_emulation.ros2_header_fbs_py.aos.ros2_emulation.Ros2Time import Ros2TimeT


class Publisher:

    def __init__(self, sender):
        self.sender = sender

    def publish(self, msg):
        self.sender.send(msg)


class Timer:

    def __init__(self, event_loop, timer, timer_period_ns):
        self.event_loop = event_loop
        self.timer = timer
        self.timer_period_ns = timer_period_ns

    def destroy(self):
        self.timer.disable()

    def cancel(self):
        self.timer.disable()

    def reset(self):
        self.timer.schedule_ns(
            self.event_loop.monotonic_now_ns() + self.timer_period_ns,
            self.timer_period_ns)


class Time:

    def __init__(self, nanoseconds):
        self.nanoseconds = nanoseconds

    def seconds_nanoseconds(self):
        nanoseconds = self.nanoseconds % 1_000_000_000
        seconds = (self.nanoseconds - nanoseconds) // 1_000_000_000
        return seconds, nanoseconds

    def to_msg(self):
        msg = Ros2TimeT()
        seconds, nseconds = self.seconds_nanoseconds()
        msg.sec = seconds
        msg.nanosec = nseconds
        return msg


class Clock:

    def __init__(self, event_loop):
        self.event_loop = event_loop

    def now(self):
        return Time(self.event_loop.realtime_now_ns())


class AosRclpyNode:
    """A class implementing part of the rclpy.node.Node API on top of AOS.

    See README.md in this folder for details."""

    def __init__(self, event_loop):
        # This is intended for general access, and eventually usage of this
        # entire class may be replaced with just this variable.
        self.event_loop = event_loop

    def get_name(self):
        return self.event_loop.name()

    def create_publisher(self, msg_type, topic, qos_profile=None):
        _ = qos_profile
        return Publisher(
            self.event_loop.make_sender(msg_type, self.convert_topic(topic)))

    def create_subscription(self, msg_type, topic, callback, qos_profile=None):
        _ = qos_profile
        self.event_loop.make_watcher(msg_type, self.convert_topic(topic),
                                     callback)

    def create_timer(self, timer_period_sec, callback, autostart=True):
        """Unlike ROS2, a timer created with autostart=True and then canceled will still be started when the EventLoop starts."""
        timer_period_ns = int(timer_period_sec * 1_000_000_000)
        result = Timer(self.event_loop, self.event_loop.add_timer(callback),
                       timer_period_ns)
        if autostart:
            self.event_loop.on_run(result.reset)
        return result

    def convert_topic(self, topic):
        if topic.startswith('/'):
            return topic
        return '/' + topic

    def get_clock(self):
        return Clock(self.event_loop)
