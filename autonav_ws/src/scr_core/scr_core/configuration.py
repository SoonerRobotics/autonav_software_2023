from scr_msgs.msg import ConfigurationInstruction
from rclpy.node import Node
import struct


GET = 0
SET = 1
GET_ACK = 2
SET_ACK = 3
GET_ALL = 4


class Configuration:
    def __init__(self, id: str, node: Node):
        self.id = id.replace("/", "")
        self.node = node
        self.subscriber = node.create_subscription(ConfigurationInstruction, "/scr/configuration", self.onConfigurationInstruction, 100)
        self.publisher = node.create_publisher(ConfigurationInstruction, "/scr/configuration", 100)

        # a map of key to map of key to byte array
        self.cache = {}
        self.cache[self.id] = {}

    def getInt(self, address: str) -> int:
        return self.getIntFrom(self.id, address)

    def getIntFrom(self, device: str, address: str) -> int:
        return int.from_bytes(self.cache[device][address], byteorder='big', signed=True)

    def setInt(self, address: str, value: int) -> None:
        self.setIntTo(self.id, address, value)

    def setIntTo(self, device: str, address: str, value: int) -> None:
        if device not in self.cache:
            self.cache[device] = {}
        self.cache[device][address] = value.to_bytes(4, byteorder='big', signed=True)
        instruction = ConfigurationInstruction()
        instruction.device = device
        instruction.opcode = SET_ACK
        instruction.address = address
        instruction.data = self.cache[device][address]
        self.publisher.publish(instruction)

    def getFloat(self, address: str) -> float:
        return self.getFloatFrom(self.id, address)

    def getFloatFrom(self, device: str, address: str) -> float:
        return struct.unpack('f', self.cache[device][address])[0]

    def setFloat(self, address: str, value: float) -> None:
        self.setFloatTo(self.id, address, value)

    def setFloatTo(self, device: str, address: str, value: float) -> None:
        if device not in self.cache:
            self.cache[device] = {}
        self.cache[device][address] = bytearray(struct.pack('f', value))
        instruction = ConfigurationInstruction()
        instruction.device = device
        instruction.opcode = SET_ACK
        instruction.address = address
        instruction.data = self.cache[device][address]
        self.publisher.publish(instruction)

    def getBool(self, address: str) -> bool:
        return self.getBoolFrom(self.id, address)

    def getBoolFrom(self, device: str, address: str) -> bool:
        return bool.from_bytes(self.cache[device][address], byteorder='big')

    def setBool(self, address: str, value: bool) -> None:
        self.setBoolTo(self.id, address, value)

    def setBoolTo(self, device: str, address: str, value: bool) -> None:
        if device not in self.cache:
            self.cache[device] = {}
        self.cache[device][address] = bytes([value])
        instruction = ConfigurationInstruction()
        instruction.device = device
        instruction.opcode = SET_ACK
        instruction.address = address
        instruction.data = self.cache[device][address]
        self.publisher.publish(instruction)

    def recache(self):
        self.cache = {}

    def onConfigurationInstruction(self, instruction: ConfigurationInstruction):
        amTarget = instruction.device == self.id

        if instruction.device not in self.cache:
            self.cache[instruction.device] = {}

        if instruction.opcode == GET and amTarget:
            response = ConfigurationInstruction()
            response.device = self.id
            response.opcode = GET_ACK
            response.address = instruction.address
            response.data = self.cache[instruction.device][instruction.address]
            self.publisher.publish(response)

        if instruction.opcode == SET and amTarget:
            self.cache[instruction.device][instruction.address] = instruction.data
            response = ConfigurationInstruction()
            response.device = self.id
            response.opcode = SET_ACK
            response.address = instruction.address
            response.data = self.cache[instruction.device][instruction.address]
            self.publisher.publish(response)

        if instruction.opcode == SET_ACK or instruction.opcode == GET_ACK:
            if instruction.device not in self.cache:
                self.cache[instruction.device] = {}
            self.cache[instruction.device][instruction.address] = instruction.data

        if instruction.opcode == GET_ALL and amTarget and self.id in self.cache:
            for address in self.cache[self.id]:
                response = ConfigurationInstruction()
                response.device = self.id
                response.opcode = GET_ACK
                response.address = address
                response.data = self.cache[self.id][address]
                self.publisher.publish(response)
