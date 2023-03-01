#!/usr/bin/env python3

import rclpy
import cv2
import time
import threading
from steamcontroller import SteamController
from steamcontroller import SteamControllerInput

from enum import IntEnum
from autonav_msgs.msg import SteamInput

from autonav_libs import Device, AutoNode, DeviceStateEnum as DeviceState, SystemStateEnum as SystemState

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



class Steamy(AutoNode):
    def __init__(self):
        super().__init__(Device.STEAM_TRANSLATOR, "autonav_steam_translator")

    def setup(self):
        self.steam_thread = threading.Thread(target=self.start_steam_controller)
        self.steam_thread.daemon = True
        self.steam_thread.start()
        self.buttons = {}
        for button in SCButtons:
            self.buttons[button] = 0
        self.switchStates = {}
        self.switchStates["MANUAL"] = False
        self.pub = self.create_publisher(SteamInput, "/autonav/joy/steam", 20)

    def start_steam_controller(self):
        try:
            sc = SteamController(callback = self.on_callback)
            if sc._handle:
                self.set_device_state(DeviceState.OPERATING)
            sc.run()
        except KeyboardInterrupt:
            self.set_device_state(DeviceState.OFF)
            sc.close()
            pass
        finally:
            self.log("Attempting to reconnect to STEAM controller")
            time.sleep(5)
            self.set_device_state(DeviceState.STANDBY)
            self.start_steam_controller()

    def on_callback(self, _, sci: SteamControllerInput):
        msg = SteamInput()
        msg.status = int(sci.status)
        msg.seq = int(sci.seq)
        msg.buttons = []
        for button in SCButtons:
            msg.buttons.append(bool(sci.buttons & button))
            if self.buttons[button] == 0 and (sci.buttons & button):
                self.buttons[button] = time.time() * 1000
            if self.buttons[button] > 0 and (sci.buttons & button) == False:
                self.buttons[button] = 0
                if button == SCButtons.START:
                    self.switchStates["MANUAL"] = False

        if (time.time() * 1000) - self.buttons[SCButtons.START] > 1250 and self.buttons[SCButtons.START] != 0 and self.switchStates["MANUAL"] == False:
            self.log("Switching to manual mode!")
            self.switchStates["MANUAL"] = True
            self.set_system_state(SystemState.MANUAL)

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
        self.pub.publish(msg)
        
def main():
    print("pre ros")
    rclpy.init()
    print("mid ros")
    rclpy.spin(Steamy())
    print("post ros")
    rclpy.shutdown()

if __name__ == "__main__":
    main()