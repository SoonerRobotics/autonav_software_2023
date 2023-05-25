#!/usr/bin/env python3

import rclpy
from scr_core.node import Node
from scr_msgs.msg import SystemState, DeviceState, Log, ConfigurationInstruction
from scr_msgs.srv import SetSystemState
from std_msgs.msg import Empty
from autonav_msgs.msg import Position, MotorFeedback, MotorInput, MotorControllerDebug, ObjectDetection, PathingDebug, GPSFeedback, IMUData, Conbus
from sensor_msgs.msg import CompressedImage
import asyncio
import websockets
import threading
import json
import base64


class BroadcastNode(Node):
	def __init__(self):
		super().__init__("autonav_display_broadcast")

		self.port = self.declare_parameter("port", 8023).get_parameter_value().integer_value
		self.host = self.declare_parameter("host", "0.0.0.0").get_parameter_value().string_value
		self.sendQueue = []
		self.sendQueueLock = threading.Lock()
		self.connectedClients = 0

		self.systemStateSubscriber = self.create_subscription(SystemState, "/scr/state/system", self.systemStateCallback, 20)
		self.deviceStateSubscriber = self.create_subscription(DeviceState, "/scr/state/device", self.deviceStateCallback, 20)
		self.broadcastPublisher = self.create_publisher(Empty, "/scr/state/broadcast", 20)
		self.logSubscriber = self.create_subscription(Log, "/scr/logging", self.logCallback, 20)
		self.configurationInstructionSubscriber = self.create_subscription(ConfigurationInstruction, "/scr/configuration", self.configurationInstructionCallback, 20)
		self.configurationInstructionPublisher = self.create_publisher(ConfigurationInstruction, "/scr/configuration", 20)

		self.positionSubscriber = self.create_subscription(Position, "/autonav/position", self.positionCallback, 20)
		self.motorFeedbackSubscriber = self.create_subscription(MotorFeedback, "/autonav/MotorFeedback", self.motorFeedbackCallback, 20)
		self.motorInputSubscriber = self.create_subscription(MotorInput, "/autonav/MotorInput", self.motorInputCallback, 20)
		self.motorControllerDebugSubscriber = self.create_subscription(MotorControllerDebug, "/autonav/MotorControllerDebug", self.motorControllerDebugCallback, 20)
		self.objectDetectionSubscriber = self.create_subscription(ObjectDetection, "/autonav/ObjectDetection", self.objectDetectionCallback, 20)
		self.pathingDebugSubscriber = self.create_subscription(PathingDebug, "/autonav/astar/debug", self.pathingDebugCallback, 20)
		self.gpsFeedbackSubscriber = self.create_subscription(GPSFeedback, "/autonav/gps", self.gpsFeedbackCallback, 20)
		self.imuDataSubscriber = self.create_subscription(IMUData, "/autonav/imu", self.imuDataCallback, 20)
		self.conbusSubscriber = self.create_subscription(Conbus, "/autonav/conbus", self.conbusCallback, 20)
		self.conbusPublisher = self.create_publisher(Conbus, "/autonav/conbus", 20)

		self.systemStateService = self.create_client(SetSystemState, "/scr/state/set_system_state")

		self.cameraSubscriber = self.create_subscription(CompressedImage, "/autonav/camera/compressed", self.cameraCallback, 20)
		self.filteredSubscriber = self.create_subscription(CompressedImage, "/autonav/cfg_space/raw/image", self.filteredCallback, 20)
		
		asyncio.get_event_loop().create_task(self.start())
	
	async def start(self):
		async with websockets.serve(self.handler, self.host, self.port):
			await asyncio.Future()

	async def consumer(self, websocket):
		async for message in websocket:
			obj = json.loads(message)
			if obj["op"] == "broadcast":
				self.broadcastPublisher.publish(Empty())

			if obj["op"] == "configuration" and "device" in obj and "opcode" in obj:
				msg = ConfigurationInstruction()
				msg.device = str(obj["device"])
				msg.opcode = int(obj["opcode"])
				msg.data = obj["data"] if "data" in obj else []
				msg.address = str(obj["address"]) if "address" in obj else ""
				self.configurationInstructionPublisher.publish(msg)

			if obj["op"] == "get_nodes":
				nodes = self.get_node_names()
				for i in range(len(nodes)):
					nodes[i] = nodes[i].replace("/", "")
				self.pushSendQueue(json.dumps({
					"op": "get_nodes_callback",
					"nodes": nodes
				}))

			if obj["op"] == "set_system_state":
				request = SetSystemState.Request()
				request.state = int(obj["state"])
				request.estop = bool(obj["estop"])
				request.mobility = bool(obj["mobility"])
				self.systemStateClient.call_async(request)

			if obj["op"] == "conbus":
				id = int(obj["id"])
				data = list(map(lambda x: int(x), obj["data"]))
				msg = Conbus()
				msg.id = id
				msg.data = data
				self.conbusPublisher.publish(msg)

	def pushSendQueue(self, message):
		if self.connectedClients == 0:
			return
			

		self.sendQueueLock.acquire()
		self.sendQueue.append(message)
		self.sendQueueLock.release()

	async def producer(self, websocket):
		while True:
			if len(self.sendQueue) > 0:
				self.sendQueueLock.acquire()
				message = self.sendQueue.pop(0)
				self.sendQueueLock.release()
				try:
					await websocket.send(message)
				except websockets.exceptions.ConnectionClosed:
					break
			else:
				await asyncio.sleep(0.05)

	async def handler(self, websocket):
		self.connectedClients += 1
			
		consumer_task = asyncio.create_task(self.consumer(websocket))
		producer_task = asyncio.create_task(self.producer(websocket))
		pending = await asyncio.wait(
			[consumer_task, producer_task],
			return_when=asyncio.FIRST_COMPLETED,
		)
		for task in pending[0]:
			task.cancel()

		self.connectedClients -= 1

	def systemStateCallback(self, msg: SystemState):
		self.pushSendQueue(json.dumps({
			"op": "data",
			"topic": "/scr/state/system",
			"state": msg.state,
			"estop": msg.estop,
			"mobility": msg.mobility,
			"mode": msg.mode
		}))

	def deviceStateCallback(self, msg: DeviceState):
		self.pushSendQueue(json.dumps({
			"op": "data",
			"topic": "/scr/state/device",
			"state": msg.state,
			"device": msg.device
		}))

	def logCallback(self, msg: Log):
		if msg.node == "autonav_display_broadcast":
			return
		self.pushSendQueue(json.dumps({
			"op": "data",
			"topic": "/scr/logging",
			"data": msg.data,
			"node": msg.node
		}))

	def configurationInstructionCallback(self, msg: ConfigurationInstruction):
		self.pushSendQueue(json.dumps({
			"op": "data",
			"topic": "/scr/configuration/instruction",
			"device": msg.device,
			"opcode": msg.opcode,
			"data": msg.data.tolist(),
			"address": msg.address
		}))

	def positionCallback(self, msg: Position):
		self.pushSendQueue(json.dumps({
			"op": "data",
			"topic": "/autonav/position",
			"x": msg.x,
			"y": msg.y,
			"theta": msg.theta,
			"latitude": msg.latitude,
			"longitude": msg.longitude
		}))

	def motorInputCallback(self, msg: MotorInput):
		self.pushSendQueue(json.dumps({
			"op": "data",
			"topic": "/autonav/MotorInput",
			"angular_velocity": msg.angular_velocity,
			"forward_velocity": msg.forward_velocity
		}))

	def motorFeedbackCallback(self, msg: MotorFeedback):
		self.pushSendQueue(json.dumps({
			"op": "data",
			"topic": "/autonav/MotorFeedback",
			"delta_x": msg.delta_x,
			"delta_y": msg.delta_y,
			"delta_theta": msg.delta_theta
		}))

	def imuDataCallback(self, msg: IMUData):
		self.pushSendQueue(json.dumps({
			"op": "data",
			"topic": "/autonav/imu",
			"accel_x": msg.accel_x,
			"accel_y": msg.accel_y,
			"accel_z": msg.accel_z,
			"angular_x": msg.angular_x,
			"angular_y": msg.angular_y,
			"angular_z": msg.angular_z,
			"yaw": msg.yaw,
			"pitch": msg.pitch,
			"roll": msg.roll
		}))

	def gpsFeedbackCallback(self, msg: GPSFeedback):
		self.pushSendQueue(json.dumps({
			"op": "data",
			"topic": "/autonav/gps",
			"latitude": msg.latitude,
			"longitude": msg.longitude,
			"altitude": msg.altitude,
			"satellites": msg.satellites,
			"is_locked": msg.is_locked,
			"gps_fix": msg.gps_fix
		}))

	def pathingDebugCallback(self, msg: PathingDebug):
		self.pushSendQueue(json.dumps({
			"op": "data",
			"topic": "/autonav/astar/debug",
			"desired_heading": msg.desired_heading,
			"desired_latitude": msg.desired_latitude,
			"desired_longitude": msg.desired_longitude,
			"distance_to_destination": msg.distance_to_destination,
			"waypoints": msg.waypoints.tolist()
		}))

	def objectDetectionCallback(self, msg: ObjectDetection):
		self.pushSendQueue(json.dumps({
			"op": "data",
			"topic": "/autonav/ObjectDetection",
			"sensor_1": msg.sensor_1,
			"sensor_2": msg.sensor_2,
			"sensor_3": msg.sensor_3
		}))

	def motorControllerDebugCallback(self, msg: MotorControllerDebug):
		self.pushSendQueue(json.dumps({
			"op": "data",
			"topic": "/autonav/MotorControllerDebug",
			"current_forward_velocity": msg.current_forward_velocity,
			"forward_velocity_setpoint": msg.forward_velocity_setpoint,
			"current_angular_velocity": msg.current_angular_velocity,
			"angular_velocity_setpoint": msg.angular_velocity_setpoint,
			"left_motor_output": msg.left_motor_output,
			"right_motor_output": msg.right_motor_output
		}))

	def cameraCallback(self, msg: CompressedImage):
		byts = msg.data.tobytes()
		base64_str = base64.b64encode(byts).decode("utf-8")

		self.pushSendQueue(json.dumps({
			"op": "data",
			"topic": "/autonav/camera/compressed",
			"format": msg.format,
			"data": base64_str
		}))

	def filteredCallback(self, msg: CompressedImage):
		byts = msg.data.tobytes()
		base64_str = base64.b64encode(byts).decode("utf-8")

		self.pushSendQueue(json.dumps({
			"op": "data",
			"topic": "/autonav/cfg_space/raw/image",
			"format": msg.format,
			"data": base64_str
		}))

	def conbusCallback(self, msg: Conbus):
		self.pushSendQueue(json.dumps({
			"op": "data",
			"topic": "/autonav/conbus",
			"id": msg.id,
			"data": msg.data.tolist()
		}))

	def transition(self, old, updated):
		return


async def spin(node: Node):
    cancel = node.create_guard_condition(lambda: None)
    def _spin(node: Node, future: asyncio.Future, event_loop: asyncio.AbstractEventLoop):
        while not future.cancelled():
            rclpy.spin_once(node)
        if not future.cancelled():
            event_loop.call_soon_threadsafe(future.set_result, None)
    event_loop = asyncio.get_event_loop()
    spin_task = event_loop.create_future()
    spin_thread = threading.Thread(target=_spin, args=(node, spin_task, event_loop))
    spin_thread.start()
    try:
        await spin_task
    except asyncio.CancelledError:
        cancel.trigger()
    spin_thread.join()
    node.destroy_guard_condition(cancel)


async def main():
	rclpy.init()
	node = BroadcastNode()
	await spin(node)
	rclpy.shutdown()

if __name__ == "__main__":
	asyncio.run(main())
