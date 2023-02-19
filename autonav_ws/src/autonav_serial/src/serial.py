#!/usr/bin/env python3

import rclpy
import can
import struct

from rclpy.node import Node
from autonav_msgs.msg import MotorInput


node: Node = None
canbus: can.Bus = None

MOTOR_CONTROL = 10
MOTOR_OFFSET = 35


def clamp(n, minn, maxn):
    return max(min(maxn, n), minn)


def onMotorInput(input: MotorInput):
    global canbus

    left_speed = int(input.left_motor * MOTOR_OFFSET)
    right_speed = int(input.right_motor * MOTOR_OFFSET)
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

    canbus = can.ThreadSafeBus(bustype = "slcan", channel = "/dev/autonav-can-835", bitrate = 100000)    

    rclpy.spin(node)
    canbus.shutdown()
    rclpy.shutdown()

if __name__ == "__main__":
    main()