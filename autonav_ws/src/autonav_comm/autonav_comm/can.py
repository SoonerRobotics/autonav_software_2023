import rclpy
import can
import struct

from autonav_msgs.msg import MotorInput

MAX_SPEED = 2.2
CAN_ID_SEND_MOTOR = 10

write_can: can.ThreadSafeBus = can.ThreadSafeBus(
	bustype="slcan",
	channel="/dev/igvc-can-835",
	bitrate=100000
)

def clamp(val, min, max):
	if val < min:
		return min
	if val > max:
		return max
	return val

def onMessage(data: MotorInput):
	left_speed = clamp(int(data.left_motor / MAX_SPEED * 127), -128, 127)
	right_speed = clamp(int(data.right_motor / MAX_SPEED * 127), -128, 127)

	packed_data = struct.pack("bbB", left_speed, right_speed, int(MAX_SPEED * 10))
	can_msg = can.Message(arbitration_id=CAN_ID_SEND_MOTOR, data=packed_data)
	try:
		write_can.send(can_msg)
	except:
		print("Failed to send CAN message")

def main(args=None):
	rclpy.init(args=args)

	node = rclpy.create_node("autonav_comm_can")
	node.create_subscription(MotorInput, "/autonav/motors/input", onMessage, 20)

	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()

if __name__ == "__main__":
	main()