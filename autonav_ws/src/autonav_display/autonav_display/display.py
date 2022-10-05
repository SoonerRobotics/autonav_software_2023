import rclpy

from autonav_msgs.msg import MotorFeedback, MotorInput

import PyQt6 as Qt
from PyQt6.QtWidgets import QApplication, QMainWindow, QLabel, QHBoxLayout, QVBoxLayout, QWidget
from PyQt6.QtGui import QFont

import time
import math
import threading

class AutoNavWindow(QMainWindow):
	def __init__(self):
		super(AutoNavWindow, self).__init__()

		rclpy.init(args=None)
		node = rclpy.create_node("autonav_display")

		node.create_subscription(MotorFeedback, "/autonav/motors/feedback", self.onMotorFeedback, 20)
		node.create_subscription(MotorInput, "/autonav/motors/input", self.onMotorInput, 20)

		self.last_motor_feedback = 0

		# Center and Horizontal Widgets
		self.middleHorizontal = QHBoxLayout()
		self.centerWidget = QWidget()
		self.centerWidget.setLayout(self.middleHorizontal)

		self.leftVertical = QVBoxLayout()
		self.middleHorizontal.addLayout(self.leftVertical, stretch=1)

		# Speed
		self.speed_label = QLabel(self)
		self.speed_label.setText(f"Speed: {0:0.01f}mph")
		self.speed_label.setFont(QFont("Arial", 32))

		# GPS
		self.gps_label = QLabel(self)
		self.gps_label.setText(f"GPS: {0:0.01f}, {0:0.01f}")
		self.gps_label.setFont(QFont("Arial", 32))

		# Motors
		self.motors_label = QLabel(self)
		self.motors_label.setText(f"Motors: {0:0.01f}l, {0:0.01f}r")
		self.motors_label.setFont(QFont("Arial", 32))

		# Add to leftern vertical layout
		self.leftVertical.addWidget(self.speed_label)
		self.leftVertical.addWidget(self.motors_label)
		self.leftVertical.addWidget(self.gps_label)

		self.setCentralWidget(self.centerWidget)
		self.setWindowTitle("AutoNav Display")
		self.showMaximized()

		self.rosThread = threading.Thread(target=rclpy.spin, args=(node,))
		self.rosThread.start()

	def ros_spin(node):
		rclpy.spin(node)
		node.destroy_node()
		rclpy.shutdown()

	def onMotorFeedback(self, data: MotorFeedback):
		if(self.last_motor_feedback == 0):
			self.last_motor_feedback = time.time()
			return
		
		speed = math.sqrt(data.delta_x ** 2 + data.delta_y ** 2) / (time.time() - self.last_motor_feedback)
		self.speed_label.setText(f"Speed: {speed:0.01f}mph")

	def onMotorInput(self, data: MotorInput):
		self.motors_label.setText(f"Motors: {(-data.left_motor / 2.2 * 127):0.01f}l, {(data.right_motor / 2.2 * 127):0.01f}r")

def main(args=None):
	app = QApplication([])
	app.setApplicationName("SCR Autonav 2022")
	window = AutoNavWindow()
	app.exec()