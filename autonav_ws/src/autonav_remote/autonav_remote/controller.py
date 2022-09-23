import rclpy

from sensor_msgs.msg import Joy
from rclpy.publisher import Publisher
from autonav_msgs.msg import MotorInput

motor_publisher: Publisher = None

# onJoyReceived - Joy - Called every time the /joy node publishes a new message
def onJoyReceived(data: Joy):
	if motor_publisher is None:
		return
	
	# Create a new MotorInput message and set the values
	motor_input = MotorInput()
	motor_input.left_motor = data.axes[1]
	motor_input.right_motor = data.axes[4]

	# Publish the message
	motor_publisher.publish(motor_input)

def main(args=None):
	global motor_publisher

	rclpy.init(args=args)

	# Create the node and subscribe to the /joy topic
	node = rclpy.create_node("autonav_comm_can")
	node.create_subscription(Joy, "/joy", onJoyReceived, 20)

	# Create the publisher for the /autonav/motors/input topic
	motor_publisher = node.create_publisher(MotorInput, "/autonav/motors/input", 20)

	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()

if __name__ == "__main__":
	main()