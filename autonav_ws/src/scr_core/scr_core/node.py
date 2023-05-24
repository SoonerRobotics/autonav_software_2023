from scr_core.state import DeviceStateEnum, SystemStateEnum
from scr_msgs.srv import SetDeviceState, SetSystemState
from scr_msgs.msg import DeviceState, SystemState, Log
from scr_core.configuration import Configuration
from scr_core.performance import Performance
from rclpy.node import Node as ROSNode
from std_msgs.msg import Empty
import time
import signal
import os


class Node(ROSNode):
    """
    A Node in the ROS graph.

    Specialized for SCR (Sooner Competitive Robotics), includes systems for state and configuration.
    """

    def __init__(self, node_name):
        super().__init__(node_name)
        self.id = node_name

        # State System
        self.config = Configuration(node_name, self)
        self.deviceStateSubscriber = self.create_subscription(DeviceState, "/scr/state/device", self.onDeviceState, 100)
        self.systemStateSubscriber = self.create_subscription(SystemState, "/scr/state/system", self.onSystemState, 100)
        self.deviceStateClient = self.create_client(SetDeviceState, "/scr/state/set_device_state")
        self.systemStateClient = self.create_client(SetSystemState, "/scr/state/set_system_state")
        self.resetSubscriber = self.create_subscription(Empty, "/scr/reset", self.onResetInternal, 100)
        self.resetPublisher = self.create_publisher(Empty, "/scr/reset", 100)
        self.logPublisher = self.create_publisher(Log, "/scr/logging", 100)

        # Configuration
        self.config = Configuration(self.id, self)
        self.deviceStates = {}
        self.state = SystemState()

        # Performance
        self.performance = Performance(self)

    def configure(self):
        pass

    def onReset(self):
        pass

    def onResetInternal(self, _):
        self.onReset()

    def reset(self):
        self.resetPublisher.publish(Empty())

    def getDeviceID(self) -> int:
        return self.id
    
    def log(self, message: str):
        log = Log()
        log.node = self.get_name()
        log.data = message
        self.logPublisher.publish(log)

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

        if self.getDeviceState() == DeviceStateEnum.OFF:
            return

        self.transition(old, state)

    def getClockNs(self) -> int:
        return time.time() * 1000000
        
    def getClockMs(self) -> int:
        return time.time() * 1000
    
    def getClockSec(self) -> float:
        return time.time()

    def onDeviceState(self, state: DeviceState):
        self.deviceStates[state.device] = state.state
        if state.device != self.id:
            return
        
        if state.state == DeviceStateEnum.STANDBY:
            self.config.recache()
            self.configure()
            self.onSystemState(self.state)

    def transition(self, old: SystemState, updated: SystemState):
        raise NotImplementedError()

    def getSystemState(self) -> SystemState:
        return self.state

    def getDeviceState(self, id: str = None) -> DeviceStateEnum:
        if id is None:
            return DeviceStateEnum.OFF if self.id not in self.deviceStates else self.deviceStates[self.id]
        return self.deviceStates[self.id] if id is None else self.deviceStates[self.id]
    
    def getDeviceStates(self):
        return self.deviceStates
