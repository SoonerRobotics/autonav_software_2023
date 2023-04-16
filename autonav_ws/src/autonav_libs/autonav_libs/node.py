from autonav_libs.state import DeviceStateEnum, SystemStateEnum
from autonav_msgs.srv import SetDeviceState, SetSystemState
from autonav_msgs.msg import DeviceState, SystemState
from autonav_libs.configuration import Configuration
from autonav_libs import hash
from rclpy.node import Node
import os
import signal


class AutoNode(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        self.id = hash(node_name)

        # State System
        self.config = Configuration(node_name, self)
        self.deviceStateSubscriber = self.create_subscription(
            DeviceState, "/autonav/state/device", self.onDeviceState, 10)
        self.systemStateSubscriber = self.create_subscription(
            SystemState, "/autonav/state/system", self.onSystemState, 10)
        self.deviceStateClient = self.create_client(
            SetDeviceState, "/autonav/state/set_device_state")
        self.systemStateClient = self.create_client(
            SetSystemState, "/autonav/state/set_system_state")

        # Configuration
        self.config = Configuration(self.id, self)
        self.deviceStates = {}
        self.state = SystemState()

    def configure(self):
        pass

    def getDeviceID(self) -> int:
        return self.id

    def setSystemStateInternal(self, state: SystemState):
        request = SetSystemState.Request()
        request.state = state.state
        request.estop = state.estop
        request.mobility = state.mobility
        self.systemStateClient.call_async(request)

    def setDeviceState(self, state: DeviceStateEnum):
        request = SetDeviceState.Request()
        request.state = state
        request.device = self.id
        self.deviceStateClient.call_async(request)

    def setSystemState(self, state: SystemStateEnum):
        self.setSystemStateInternal(SystemState(
            state = state,
            estop = self.state.estop,
            mobility = self.state.mobility
        ))
    
    def setEStop(self, state: bool):
        self.setSystemStateInternal(SystemState(
            state = self.state.state,
            estop = state,
            mobility = self.state.mobility
        ))

    def setMobility(self, state: bool):
        self.setSystemStateInternal(SystemState(
            state = self.state.state,
            estop = self.state.estop,
            mobility = state
        ))

    def onSystemState(self, state: SystemState):
        if state.state == SystemStateEnum.SHUTDOWN:
            os.kill(os.getpid(), signal.SIGINT)

        old = self.state
        self.state = state
        self.transition(old, state)

    def onDeviceState(self, state: DeviceState):
        self.deviceStates[state.device] = state.state
        if state.device != self.id:
            return
        
        if state.state == DeviceStateEnum.STANDBY:
            self.config.recache()
            self.configure()
            return

    def transition(self, _: SystemState, updated: SystemState):
        if updated.state == SystemStateEnum.DISABLED:
            return
        
        if updated.state == SystemStateEnum.DISABLED and self.getDeviceState() == DeviceStateEnum.OPERATING:
            self.setDeviceState(DeviceStateEnum.READY)
            return
        
        if (updated.state == SystemStateEnum.MANUAL or updated.state == SystemStateEnum.AUTONOMOUS) and self.getDeviceState() == DeviceStateEnum.READY:
            self.setDeviceState(DeviceStateEnum.OPERATING)
            return

    def getSystemState(self) -> SystemState:
        return self.state

    def getDeviceState(self, id: int = None) -> DeviceStateEnum:
        return self.deviceStates[id] if id else self.deviceStates[self.id]
    
    def getDeviceStates(self):
        return self.deviceStates
