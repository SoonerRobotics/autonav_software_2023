from scr_core.state import DeviceStateEnum, SystemStateEnum
from scr_msgs.srv import SetDeviceState, SetSystemState
from scr_msgs.msg import DeviceState, SystemState, Log
from scr_core.configuration import Configuration
from rclpy.node import Node as ROSNode
from std_msgs.msg import Empty
from scr_core import hash
import signal
import os


class Node(ROSNode):
    """
    A Node in the ROS graph.

    Specialized for SCR (Sooner Competitive Robotics), includes systems for state and configuration.
    """

    def __init__(self, node_name):
        super().__init__(node_name)
        self.id = hash(node_name)

        # State System
        self.config = Configuration(node_name, self)
        self.deviceStateSubscriber = self.create_subscription(DeviceState, "/scr/state/device", self.onDeviceState, 20)
        self.systemStateSubscriber = self.create_subscription(SystemState, "/scr/state/system", self.onSystemState, 20)
        self.deviceStateClient = self.create_client(SetDeviceState, "/scr/state/set_device_state")
        self.systemStateClient = self.create_client(SetSystemState, "/scr/state/set_system_state")
        self.resetSubscriber = self.create_subscription(Empty, "/scr/reset", self.onResetInternal, 1)
        self.resetPublisher = self.create_publisher(Empty, "/scr/reset", 1)
        self.logPublisher = self.create_publisher(Log, "/scr/logging", 20)

        # Configuration
        self.config = Configuration(self.id, self)
        self.deviceStates = {}
        self.state = SystemState()

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
        self.transition(old, state)
        
    def getClockMs(self) -> int:
        return self.get_clock().now().nanoseconds / 1000000
    
    def getClockSec(self) -> float:
        return self.get_clock().now().nanoseconds / 1000000000

    def onDeviceState(self, state: DeviceState):
        self.deviceStates[state.device] = state.state
        if state.device != self.id:
            return
        
        if state.state == DeviceStateEnum.STANDBY:
            self.config.recache()
            self.configure()
            return

    def transition(self, old: SystemState, updated: SystemState):
        raise NotImplementedError()

    def getSystemState(self) -> SystemState:
        return self.state

    def getDeviceState(self, id: int = None) -> DeviceStateEnum:
        return self.deviceStates[id] if id else self.deviceStates[self.id]
    
    def getDeviceStates(self):
        return self.deviceStates
