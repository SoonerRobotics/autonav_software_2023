#!/usr/bin/env python3

import rclpy
import threading
import can
import struct
import serial

from rclpy.node import Node
from std_msgs.msg._string import String

from autonav_msgs.msg._motor_input import MotorInput

node: Node = None
canbus: can.Bus = None

ESTOP_ID = 0
MOBSTOP_ID = 1
MOBSTART_ID = 9
MOTOR_CONTROL = 10
VELOCITY_FEEDBACK_ID = 11
ODOMETRY_FEEDBACK_ID = 14

MAX_SPEED = 2.2

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


def clamp(n, minn, maxn):
    return max(min(maxn, n), minn)


def onMotorInput(input: MotorInput):
    global canbus

    left_speed = int(input.left_motor * 10000)
    right_speed = int(input.right_motor * 10000)
    print(f"Sending motor message: {left_speed} {right_speed}")
    packed_data = struct.pack("hh", left_speed, right_speed)

    can_msg = can.Message(arbitration_id=MOTOR_CONTROL, data=packed_data)

    try:
        canbus.send(can_msg)
    except can.CanError:
        print("Failed to send motor message :(")

def main():
    global node, canbus

    rclpy.init()

    node = rclpy.create_node("autonav_serial")
    node.create_subscription(MotorInput, "/autonav/MotorInput", onMotorInput, 10)

    canbus = can.ThreadSafeBus(bustype = "slcan", channel = "/dev/ttyACM1", bitrate = 100000)    
    # can_thread = CANReadThread()
    # can_thread.start()

    rclpy.spin(node)
    canbus.shutdown()
    rclpy.shutdown()

if __name__ == "__main__":
    main()