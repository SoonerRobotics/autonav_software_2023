import rclpy
import threading
import can
import struct

from rclpy.node import Node
from std_msgs.msg._string import String

node: Node = None
canbus: can.Bus = None

ESTOP_ID = 0
MOBSTOP_ID = 1
MOBSTART_ID = 9
VELOCITY_FEEDBACK_ID = 11
ODOMETRY_FEEDBACK_ID = 14

class CANReadThread(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self, daemon = True)

        self.odom_publisher = node.create_publisher("/autonav/deltaodom", String, 10)

    def run(self):
        while True:
            msg = canbus.recv(timeout = 1)

            if msg is None:
                continue

            if msg.arbitration_id == VELOCITY_FEEDBACK_ID:
                left_speed, right_speed, max_speed = struct.unpack("bbB", msg.data)

            if msg.arbitration_id == ODOMETRY_FEEDBACK_ID:
                delta_theta, delta_y, delta_x = struct.unpack("hhh", msg.data)

            if msg.arbitration_id ==  ESTOP_ID or msg.arbitration_id == MOBSTOP_ID:
                print(f"Received Stop -> {msg.arbitration_id}")
            
            if msg.arbitration_id == MOBSTART_ID:
                print("Received MobStart")


def main():
    global node, canbus

    rclpy.init()

    node = rclpy.create_node("autonav_serial")

    canbus = can.ThreadSafeBus(bustype = "slcan", channel = "/dev/autonav-can-835", bitrate = 100000)    
    can_thread = CANReadThread()
    can_thread.start()

    rclpy.spin(node)
    rclpy.shutdown()