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
    RPAD = 0b00000100000000000000000000000000
    LPAD = 0b00000010000000000000000000000000
    RGRIP = 0b00000001000000000000000000000000
    LGRIP = 0b00000000100000000000000000000000
    START = 0b00000000010000000000000000000000
    STEAM = 0b00000000001000000000000000000000
    BACK = 0b00000000000100000000000000000000
    A = 0b00000000000000001000000000000000
    X = 0b00000000000000000100000000000000
    B = 0b00000000000000000010000000000000
    Y = 0b00000000000000000001000000000000
    LB = 0b00000000000000000000100000000000
    RB = 0b00000000000000000000010000000000
    LT = 0b00000000000000000000001000000000
    RT = 0b00000000000000000000000100000000


class SteamTranslationNode(AutoNode):
    def __init__(self):
        super().__init__(Device.STEAM_TRANSLATOR, "autonav_steam_translator")

    def setup(self):
        self.m_steamThread = threading.Thread(
            target=self.start_steam_controller)
        self.m_steamThread.daemon = True
        self.m_steamThread.start()
        self.m_Buttons = {}
        for button in SCButtons:
            self.m_Buttons[button] = 0
        self.m_debounce = {}
        self.m_debounce["MANUAL"] = False
        self.m_joyPublisher = self.create_publisher(
            SteamInput, "/autonav/joy/steam", 20)

    def start_steam_controller(self):
        try:
            sc = SteamController(callback=self.on_callback)
            if sc._handle:
                self.setDeviceState(DeviceState.OPERATING)
            sc.run()
        except KeyboardInterrupt:
            self.setDeviceState(DeviceState.OFF)
            sc.close()
            pass
        finally:
            time.sleep(5)
            self.setDeviceState(DeviceState.STANDBY)
            self.start_steam_controller()

    def on_callback(self, _, sci: SteamControllerInput):
        msg = SteamInput()
        msg.status = int(sci.status)
        msg.seq = int(sci.seq)
        msg.buttons = []
        for button in SCButtons:
            msg.buttons.append(bool(sci.buttons & button))
            if self.m_Buttons[button] == 0 and (sci.buttons & button):
                self.m_Buttons[button] = time.time() * 1000
            if self.m_Buttons[button] > 0 and (sci.buttons & button) == False:
                self.m_Buttons[button] = 0
                if button == SCButtons.START:
                    self.m_debounce["MANUAL"] = False

        if (time.time() * 1000) - self.m_Buttons[SCButtons.START] > 1250 and self.m_Buttons[SCButtons.START] != 0 and self.m_debounce["MANUAL"] == False:
            self.m_debounce["MANUAL"] = True
            self.setSystemState(SystemState.MANUAL)

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
        self.m_joyPublisher.publish(msg)


def main():
    rclpy.init()
    rclpy.spin(SteamTranslationNode())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
