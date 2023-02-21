#!/usr/bin/env python3

import rclpy
import cv2
import threading
from steamcontroller import SteamController
from steamcontroller import SteamControllerInput

from autonav_msgs.msg import SteamInput
from rclpy.publisher import Publisher

pub: Publisher = None

def on_callback(_, sci: SteamControllerInput):
    msg = SteamInput()
    msg.trackpad_x = float(sci.rpad_x)
    msg.trackpad_y = float(sci.rpad_y)
    msg.joystick_x = float(sci.lpad_x)
    msg.joystick_y = float(sci.lpad_y)
    pub.publish(msg)


def start_steam_controller():
    sc = SteamController(callback = on_callback)
    sc.run()


def main(args = None):
    global pub
    rclpy.init()

    node = rclpy.create_node("autonav_manual_steam")
    pub = node.create_publisher(SteamInput, "/autonav/joy/steam", 20)

    steam_thread = threading.Thread(target=start_steam_controller)
    steam_thread.start()

    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()