import signal
import time
import struct
import os
from enum import IntEnum
from rclpy.node import Node
from autonav_msgs.msg import ConBusInstruction, Log, DeviceState, SystemState, PerfResult
from autonav_msgs.srv import SetDeviceState, SetSystemState


class DeviceStateEnum(IntEnum):
    OFF = 0
    STANDBY = 1
    READY = 2
    OPERATING = 3


class SystemStateEnum(IntEnum):
    DISABLED = 0
    AUTONOMOUS = 1
    MANUAL = 2
    SHUTDOWN = 3


class ConbusOpcode(IntEnum):
    READ = 0,
    READ_ACK = 1
    WRITE = 2
    WRITE_ACK = 3
    READ_ALL = 4


class LogLevel(IntEnum):
    DEBUG = 0
    INFO = 1
    WARNING = 2
    ERROR = 3
    CRITICAL = 4


def hash(s: str):
    h = 5381
    c = 0

    for b in range(len(s)):
        c = ord(s[b])
        h = ((h << 5) + h) + c

    return h & 0xFFFFFFFFFFFF


def clamp(n, minn, maxn):
    return max(min(maxn, n), minn)


class AutoNode(Node):
    def __init__(self, node_name):
        super().__init__(node_name)

        self.id = hash(node_name)
        self.config = Conbus(node_name, self)
        self.m_logPublisher = self.create_publisher(Log, "/autonav/logging", 10)
        self.m_deviceStateSubscriber = self.create_subscription(DeviceState, "/autonav/state/device", self.onDeviceStateChanged, 10)
        self.m_systemStateSubscriber = self.create_subscription(SystemState, "/autonav/state/system", self.onSystemStateChanged, 10)
        self.m_deviceStateClient = self.create_client(SetDeviceState, "/autonav/state/set_device_state")
        self.m_systemStateClient = self.create_client(SetSystemState, "/autonav/state/set_system_state")
        self.log(f"Created node {node_name} with id {self.id}")

        self.m_systemState = SystemStateEnum.DISABLED
        self.m_isSimulator = False
        self.m_deviceStates = {}
        self.m_deviceStates[self.id] = DeviceStateEnum.OFF
        self.m_initializeTimer = self.create_timer(0.3, self.onInitializeTimer)

    def setup(self):
        pass

    def operate(self):
        pass

    def deoperate(self):
        pass

    def shutdown(self):
        pass

    def onSystemStateUpdated(self):
        pass

    def onDeviceStateUpdated(self):
        pass

    def getSystemState(self):
        return self.m_systemState

    def getDeviceState(self, device: str = None):
        dev = self.id if device is None else hash(device)
        return self.m_deviceStates[dev]

    def onInitializeTimer(self):
        if self.getDeviceState() != DeviceStateEnum.OFF:
            self.m_initializeTimer.cancel()
            return

        self.setDeviceState(DeviceStateEnum.STANDBY)

    def onSystemStateChanged(self, state: SystemState):
        originalState = self.getSystemState()
        self.m_systemState = SystemStateEnum(state.state)
        self.m_isSimulator = state.is_simulator
        
        if self.m_systemState != originalState:
            self.onSystemStateUpdated()

        if self.m_systemState == SystemStateEnum.SHUTDOWN:
            os.kill(os.getpid(), signal.SIGKILL)

    def onDeviceStateChanged(self, state: DeviceState):
        originalState = self.getDeviceState()
        self.m_deviceStates[state.device] = DeviceStateEnum(state.state)
        if state.device != self.id:
            return

        newState = self.getDeviceState()
        if newState == originalState:
            return

        self.onDeviceStateUpdated()
        if newState == DeviceStateEnum.STANDBY and originalState == DeviceStateEnum.OFF:
            self.setup()

        if newState == DeviceStateEnum.OPERATING and originalState == DeviceStateEnum.READY:
            self.operate()

        if newState == DeviceStateEnum.READY and originalState == DeviceStateEnum.OPERATING:
            self.deoperate()

    def setDeviceState(self, state: DeviceStateEnum):
        request = SetDeviceState.Request()
        request.device = self.id
        request.state = state.value
        self.m_deviceStateClient.call_async(request)
        return False

    def setSystemState(self, state: SystemStateEnum):
        if self.m_systemState == state:
            return True
        request = SetSystemState.Request()
        request.state = state.value
        self.m_systemStateClient.call_async(request)
        return False

    def log(self, message: str, level: LogLevel = LogLevel.INFO, file: str = None, skipFile=False, skipConsole=False):
        if not skipFile:
            if file is None:
                file = self.get_name()

            msg = Log()
            msg.data = message
            msg.file = file
            self.m_logPublisher.publish(msg)

        if not skipConsole:
            if level == LogLevel.DEBUG:
                self.get_logger().debug(message)
            if level == LogLevel.INFO:
                self.get_logger().info(message)
            if level == LogLevel.WARNING:
                self.get_logger().warn(message)
            if level == LogLevel.ERROR:
                self.get_logger().error(message)
            if level == LogLevel.CRITICAL:
                self.get_logger().error(message)


class Conbus:
    def __init__(self, node_name: str, node: Node):
        self.id = hash(node_name)
        self.registers = {}

        self.publisher = node.create_publisher(ConBusInstruction, "/autonav/conbus", 10)
        self.subscriber = node.create_subscription(ConBusInstruction, "/autonav/conbus", self.on_conbus_instruction, 10)

    def intToBytes(self, data: int):
        byts = data.to_bytes(4, byteorder="big", signed=True)
        byts = bytes([0]) + byts
        return byts

    def floatToBytes(self, data: float):
        byts = bytearray(struct.pack('f', data))
        return bytes([0]) + byts

    def boolToBytes(self, data: bool):
        if data:
            return bytes([0, 1])
        else:
            return bytes([0, 0])

    def write(self, address: int, data: bytes, dontPublish=False):
        if self.id not in self.registers:
            self.registers[self.id] = {}
        self.registers[self.id][address] = data

        if not dontPublish:
            msg = ConBusInstruction()
            msg.device = self.id
            msg.address = address
            msg.data = data
            msg.opcode = ConbusOpcode.WRITE_ACK.value
            self.publisher.publish(msg)

    def writeFloat(self, address: int, data: float, dontPublish=False):
        self.write(address, self.floatToBytes(data), dontPublish)

    def writeInt(self, address: int, data: int, dontPublish=False):
        self.write(address, self.intToBytes(data), dontPublish)

    def writeBool(self, address: int, data: bool, dontPublish=False):
        self.write(address, self.boolToBytes(data), dontPublish)

    def readBytes(self, address: int):
        if self.id not in self.registers:
            self.registers[self.id] = {}

        if address not in self.registers[self.id]:
            return None

        return self.registers[self.id][address]

    def readInt(self, address: int):
        byts = self.readBytes(address)

        if byts is None:
            return None

        if len(byts) == 5:
            # Convert last 4 bytes from big endian to int
            return int.from_bytes(byts[1:], byteorder="big", signed=True)

        raise Exception("Invalid integer size")

    def readBool(self, address: int):
        return self.readBytes(address)[1] == 1

    def readFloat(self, address: int):
        byts = self.readBytes(address)

        if byts is None:
            return None

        if len(byts) == 5:
            return struct.unpack('f', byts[1:])[0]

        raise Exception("Invalid float size")

    def writeTo(self, device: str, address: int, data: bytes):
        msg = ConBusInstruction()
        msg.device = hash(device)
        msg.address = address
        msg.data = data
        msg.opcode = ConbusOpcode.WRITE
        self.publisher.publish(msg)

    def on_conbus_instruction(self, instruction: ConBusInstruction):
        data = self.readBytes(instruction.address)
        if instruction.opcode == ConbusOpcode.READ.value and instruction.device == self.id:
            if data is None:
                return
            msg = ConBusInstruction()
            msg.device = self.id
            msg.address = instruction.address
            msg.data = data
            msg.opcode = ConbusOpcode.READ_ACK.value
            self.publisher.publish(msg)

        if instruction.opcode == ConbusOpcode.READ_ACK.value:
            if instruction.device not in self.registers:
                self.registers[instruction.device] = {}

            self.registers[instruction.device][instruction.address] = instruction.data

        if instruction.opcode == ConbusOpcode.WRITE.value and instruction.device == self.id:
            self.write(instruction.address, instruction.data)

        if instruction.opcode == ConbusOpcode.WRITE_ACK.value:
            if instruction.device not in self.registers:
                self.registers[instruction.device] = {}

            self.registers[instruction.device][instruction.address] = instruction.data

        if instruction.opcode == ConbusOpcode.READ_ALL.value and instruction.device == self.id:
            if self.id not in self.registers:
                return

            for key in self.registers[self.id]:
                msg = ConBusInstruction()
                msg.device = self.id
                msg.address = key
                msg.data = self.registers[self.id][key]
                msg.opcode = ConbusOpcode.READ_ACK.value
                self.publisher.publish(msg)