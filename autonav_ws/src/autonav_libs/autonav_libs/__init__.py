import signal
import time
import struct
import os
from enum import IntEnum
from rclpy.node import Node
from autonav_msgs.msg import ConBusInstruction, Log, DeviceState, SystemState, PerfResult
from autonav_msgs.srv import SetDeviceState, SetSystemState


class Device(IntEnum):
    STEAM_TRANSLATOR = 100
    MANUAL_CONTROL_STEAM = 101
    MANUAL_CONTROL_XBOX = 102
    DISPLAY_NODE = 103
    SERIAL_IMU = 104
    SERIAL_CAN = 105
    LOGGING = 106
    CAMERA_TRANSLATOR = 107,
    IMAGE_TRANSFORMER = 108,
    PARTICLE_FILTER = 109,
    LOGGING_COMBINED = 110


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


class AutoNode(Node):
    def __init__(self, device: Device, node_name):
        super().__init__(node_name)

        self.config = Conbus(device, self)
        self.per = Performance(device, self)
        self.device = device
        self.m_logPublisher = self.create_publisher(Log, "/autonav/logging", 10)
        self.m_deviceStateSubscriber = self.create_subscription(DeviceState, "/autonav/state/device", self.onDeviceStateChanged, 10)
        self.m_systemStateSubscriber = self.create_subscription(SystemState, "/autonav/state/system", self.onSystemStateChanged, 10)
        self.m_deviceStateClient = self.create_client(SetDeviceState, "/autonav/state/set_device_state")
        self.m_systemStateClient = self.create_client(SetSystemState, "/autonav/state/set_system_state")

        self.m_systemState = SystemStateEnum.DISABLED
        self.m_isSimulator = False
        self.m_deviceStates = {}
        self.m_deviceStates[device] = DeviceStateEnum.OFF
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

    def getDeviceState(self, device: Device = None):
        device = self.device if device is None else device
        return self.m_deviceStates[device]

    def onInitializeTimer(self):
        if self.getDeviceState() != DeviceStateEnum.OFF:
            self.m_initializeTimer.cancel()
            return

        self.setDeviceState(DeviceStateEnum.STANDBY)

    def onSystemStateChanged(self, state: SystemState):
        self.m_systemState = SystemStateEnum(state.state)
        self.m_isSimulator = state.is_simulator
        self.onSystemStateUpdated()

        if self.m_systemState == SystemStateEnum.SHUTDOWN:
            os.kill(os.getpid(), signal.SIGKILL)

    def onDeviceStateChanged(self, state: DeviceState):
        originalState = self.getDeviceState()
        self.m_deviceStates[Device(state.device)
                            ] = DeviceStateEnum(state.state)
        if state.device != self.device.value:
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
        request.device = self.device.value
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



class Performance:
    def __init__(self, device: Device, node: Node):
        self.device = device
        self.functions = {}
        self.results = {}
        
        self.publisher = node.create_publisher(PerfResult, "/autonav/performance", 10)
        
    def start(self, function: int):
        self.functions[function] = time.time()
        
    def end(self, function: int):
        if function not in self.functions:
            return
        
        if function not in self.results:
            self.results[function] = []
        
        msg = PerfResult()
        msg.device = self.device.value
        msg.function = function
        msg.time = time.time() - self.functions[function]
        self.results[function].append(msg)
        msg.average = sum([x.time for x in self.results[function]]) / len(self.results[function])
        msg.max = max([x.time for x in self.results[function]])
        msg.min = min([x.time for x in self.results[function]])
        self.publisher.publish(msg)
        
        del self.functions[function]


MAX_DEVICE_ID = 200


class Conbus:
    def __init__(self, device: Device, node: Node):
        self.device = device
        self.registers = {}

        self.publisher = node.create_publisher(ConBusInstruction, "/autonav/conbus", 10)
        self.subscriber = node.create_subscription(ConBusInstruction, "/autonav/conbus", self.on_conbus_instruction, 10)

    def intToBytes(self, data: int):
        byts = data.to_bytes(4, byteorder="big", signed=True)
        byts = bytes([0]) + byts
        return byts

    def floatToBytes(self, data: float):
        # Use a struct to convert the float to a byte array
        byts = struct.pack('>f', data)
        return byts

    def boolToBytes(self, data: bool):
        if data:
            return bytes([0, 1])
        else:
            return bytes([0, 0])

    def write(self, address: int, data: bytes, dontPublish=False):
        if self.device.value not in self.registers:
            self.registers[self.device.value] = {}
        self.registers[self.device.value][address] = data

        if not dontPublish:
            msg = ConBusInstruction()
            msg.device = self.device.value
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
        if self.device.value not in self.registers:
            self.registers[self.device.value] = {}

        if address not in self.registers[self.device.value]:
            return None

        return self.registers[self.device.value][address]

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
            # Convert last 4 bytes from big endian to int
            return struct.unpack('>f', byts[1:])[0]

        raise Exception("Invalid float size")

    def writeTo(self, device: Device, address: int, data: bytes):
        msg = ConBusInstruction()
        msg.device = device
        msg.address = address
        msg.data = data
        msg.opcode = ConbusOpcode.WRITE
        self.publisher.publish(msg)

    def on_conbus_instruction(self, instruction: ConBusInstruction):
        data = self.readBytes(instruction.address)
        if instruction.opcode == ConbusOpcode.READ.value and instruction.device == self.device.value:
            if data is None:
                return
            msg = ConBusInstruction()
            msg.device = self.device.value
            msg.address = instruction.address
            msg.data = data
            msg.opcode = ConbusOpcode.READ_ACK.value
            self.publisher.publish(msg)

        if instruction.opcode == ConbusOpcode.READ_ACK.value:
            if instruction.device not in self.registers:
                self.registers[instruction.device] = {}

            self.registers[instruction.device][instruction.address] = instruction.data

        if instruction.opcode == ConbusOpcode.WRITE.value and instruction.device == self.device.value:
            self.write(instruction.address, instruction.data)

        if instruction.opcode == ConbusOpcode.WRITE_ACK.value:
            if instruction.device not in self.registers:
                self.registers[instruction.device] = {}

            self.registers[instruction.device][instruction.address] = instruction.data

        if instruction.opcode == ConbusOpcode.READ_ALL.value and instruction.device == self.device.value:
            if self.device.value not in self.registers:
                return

            for key in self.registers[self.device.value]:
                msg = ConBusInstruction()
                msg.device = self.device.value
                msg.address = key
                msg.data = self.registers[self.device.value][key]
                msg.opcode = ConbusOpcode.READ_ACK.value
                self.publisher.publish(msg)
