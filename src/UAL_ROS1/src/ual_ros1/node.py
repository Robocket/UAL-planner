"""A narrow rospy facade shared by the algorithm-oriented Python nodes."""

from types import SimpleNamespace

import rospy


class ReliabilityPolicy:
    RELIABLE = "reliable"
    BEST_EFFORT = "best_effort"


class DurabilityPolicy:
    VOLATILE = "volatile"
    TRANSIENT_LOCAL = "transient_local"


class HistoryPolicy:
    KEEP_LAST = "keep_last"


class QoSProfile:
    def __init__(self, depth=1, reliability=None, durability=None, history=None):
        self.depth = int(depth)
        self.reliability = reliability
        self.durability = durability
        self.history = history


qos_profile_sensor_data = QoSProfile(
    depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)


class _Parameter:
    def __init__(self, value):
        self.value = value


class _Logger:
    @staticmethod
    def info(message, *args, **kwargs):
        rospy.loginfo(message, *args)

    @staticmethod
    def warning(message, *args, **kwargs):
        throttle = kwargs.get("throttle_duration_sec")
        if throttle is None:
            rospy.logwarn(message, *args)
        else:
            rospy.logwarn_throttle(float(throttle), message, *args)

    @staticmethod
    def error(message, *args, **kwargs):
        rospy.logerr(message, *args)

    @staticmethod
    def debug(message, *args, **kwargs):
        rospy.logdebug(message, *args)


class _Clock:
    @staticmethod
    def now():
        return SimpleNamespace(to_msg=rospy.Time.now)


class Node:
    """Expose only the node operations used by this repository."""

    def __init__(self, name):
        self.name = name
        self._defaults = {}
        self._resources = []

    def declare_parameter(self, name, default):
        self._defaults[name] = default
        return _Parameter(rospy.get_param("~" + name, default))

    def get_parameter(self, name):
        if name not in self._defaults and not rospy.has_param("~" + name):
            raise KeyError("undeclared private parameter: " + name)
        return _Parameter(rospy.get_param("~" + name, self._defaults.get(name)))

    def create_publisher(self, message_type, topic, qos):
        depth = qos.depth if isinstance(qos, QoSProfile) else int(qos)
        latch = (
            isinstance(qos, QoSProfile)
            and qos.durability == DurabilityPolicy.TRANSIENT_LOCAL
        )
        publisher = rospy.Publisher(
            topic, message_type, queue_size=max(1, depth), latch=latch)
        self._resources.append(publisher)
        return publisher

    def create_subscription(self, message_type, topic, callback, qos):
        depth = qos.depth if isinstance(qos, QoSProfile) else int(qos)
        subscriber = rospy.Subscriber(
            topic, message_type, callback, queue_size=max(1, depth),
            tcp_nodelay=True)
        self._resources.append(subscriber)
        return subscriber

    def create_timer(self, period_sec, callback):
        timer = rospy.Timer(
            rospy.Duration.from_sec(float(period_sec)), lambda _event: callback())
        self._resources.append(timer)
        return timer

    @staticmethod
    def get_logger():
        return _Logger()

    @staticmethod
    def get_clock():
        return _Clock()

    @staticmethod
    def count_publishers(topic):
        resolved = rospy.resolve_name(topic)
        return sum(
            1 for name, _message_type in rospy.get_published_topics()
            if rospy.resolve_name(name) == resolved)

    def destroy_node(self):
        for resource in reversed(self._resources):
            if hasattr(resource, "shutdown"):
                resource.shutdown()
            elif hasattr(resource, "unregister"):
                resource.unregister()
        self._resources.clear()
