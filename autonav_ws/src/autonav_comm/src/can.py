import rclpy

from std_msgs.msg import String

node = None
publisher = None

def onMessage(data: String):
	print(data)
	publisher.publish(data)

def main(args=None):
	global publisher
	rclpy.init(args=args)

	node = rclpy.create_node("autonav_comm_can")

	publisher = node.create_publisher(String, "/autonav/echo", 20)
	node.create_subscription(String, "/autonav/debug", onMessage, 20)

	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()

if __name__ == "__main__":
	main()