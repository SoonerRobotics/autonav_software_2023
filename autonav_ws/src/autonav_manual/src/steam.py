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
    msg.status = int(sci.status)
    msg.seq = int(sci.seq)
    msg.buttons = int(sci.buttons)
    msg.ltrig = float(sci.ltrig) / 255
    msg.rtrig = float(sci.rtrig) / 255
    msg.lpad_x = float(sci.lpad_x) / 32768
    msg.lpad_y = float(sci.lpad_y) / 32768
    msg.rpad_x = float(sci.rpad_x) / 32768
    msg.rpad_y = float(sci.rpad_y) / 32768
    msg.gpitch = float(sci.gpitch)
    msg.groll = float(sci.groll)
    msg.gyaw = float(sci.gyaw)
    msg.q1 = float(sci.q1)
    msg.q1 = float(sci.q2)
    msg.q1 = float(sci.q3)
    msg.q1 = float(sci.q4)
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
    steam_thread.daemon = True
    steam_thread.start()

    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()