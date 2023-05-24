from scr_msgs.msg import PerformanceResult
from rclpy.node import Node


class Performance:
    def __init__(self, node: Node):
        self.node = node
        self.publisher = node.create_publisher(PerformanceResult, "/scr/performance", 100)

        self.history = {}
        self.timers = {}
        self.minimum = 0
        self.maximum = 0

    def start(self, name):
        self.timers[name] = self.node.get_clock().now().nanoseconds

    def end(self, name):
        if name not in self.timers:
            return

        if name not in self.history:
            self.history[name] = []

        duration = self.node.get_clock().now().nanoseconds - self.timers[name]
        self.history[name].append(duration)

        if len(self.history[name]) > 500:
            self.history[name].pop(0)

        if duration < self.minimum or self.minimum == 0:
            self.minimum = duration

        if duration > self.maximum:
            self.maximum = duration

        self.publish(name)

    def publish(self, name):
        msg = PerformanceResult()
        msg.key = name
        msg.average = sum(self.history[name]) // len(self.history[name]) // 1000000
        msg.minimum = self.minimum // 1000000
        msg.maximum = self.maximum // 1000000
        msg.latest = self.history[name][-1] // 1000000
        msg.latest_ns = self.history[name][-1]
        msg.node = self.node.get_name()
        self.publisher.publish(msg)