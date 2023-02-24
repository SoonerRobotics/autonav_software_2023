#!/usr/bin/env python3

import rclpy
import cv2
import time
import threading
from steamcontroller import SteamController
from steamcontroller import SteamControllerInput

from enum import IntEnum
from autonav_msgs.msg import SteamInput
from rclpy.publisher import Publisher

pub: Publisher = None

class SCButtons(IntEnum):
    RPADTOUCH = 0b00010000000000000000000000000000
    LPADTOUCH = 0b00001000000000000000000000000000
    RPAD      = 0b00000100000000000000000000000000
    LPAD      = 0b00000010000000000000000000000000
    RGRIP     = 0b00000001000000000000000000000000
    LGRIP     = 0b00000000100000000000000000000000
    START     = 0b00000000010000000000000000000000
    STEAM     = 0b00000000001000000000000000000000
    BACK      = 0b00000000000100000000000000000000
    A         = 0b00000000000000001000000000000000
    X         = 0b00000000000000000100000000000000
    B         = 0b00000000000000000010000000000000
    Y         = 0b00000000000000000001000000000000
    LB        = 0b00000000000000000000100000000000
    RB        = 0b00000000000000000000010000000000
    LT        = 0b00000000000000000000001000000000
    RT        = 0b00000000000000000000000100000000

def on_callback(_, sci: SteamControllerInput):
    msg = SteamInput()
    msg.status = int(sci.status)
    msg.seq = int(sci.seq)
    msg.buttons = []
    for button in SCButtons:
        msg.buttons.append(bool(sci.buttons & button))
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
    try:
        sc = SteamController(callback = on_callback)
        sc.run()
    except KeyboardInterrupt:
        sc.close()
        pass
    except ValueError:
        time.sleep(5)
        start_steam_controller()


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