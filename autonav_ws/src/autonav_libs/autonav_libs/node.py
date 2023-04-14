from autonav_msgs.srv import SetDeviceState, SetSystemState
from autonav_msgs.msg import DeviceState, SystemState
from autonav_libs.configuration import Configuration
from autonav_libs.state import DeviceStateEnum
from rclpy.node import Node


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

    def setSystemState(self, state: SystemState):
        request = SetSystemState.Request()
        request.state = state
        self.systemStateClient.call_async(request)

    def setDeviceState(self, state: DeviceStateEnum):
        request = SetDeviceState.Request()
        request.state = state
        request.device = self.id
        self.deviceStateClient.call_async(request)

    def onSystemState(self, state: SystemState):
        self.state = state

    def onDeviceState(self, state: DeviceState):
        if state.device != self.id:
            self.deviceStates[state.device] = state.state
            return
        
        if state.state == DeviceStateEnum.STANDBY:
            self.config.recache()
            self.configure()
            self.deviceStates[state.device] = state.state
            return

        if self.transition(state.state):
            self.deviceStates[state.device] = state.state

    def transition(self, _: SystemState):
        return True

    def getSystemState(self) -> SystemState:
        return self.state

    def getDeviceState(self, id: int = None) -> DeviceStateEnum:
        return self.deviceStates[id] if id else self.deviceStates[self.id]
