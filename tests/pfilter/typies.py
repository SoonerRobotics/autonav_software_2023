class Feedback:
	def __init__(self, delta_x, delta_y, delta_theta) -> None:
		self.delta_x = delta_x
		self.delta_y = delta_y
		self.delta_theta = delta_theta
  
class GPS:
	def __init__(self, latitude, longitude) -> None:
		self.latitude = latitude
		self.longitude = longitude
  
	def __str__(self) -> str:
		return f'GPS({self.latitude}, {self.longitude})'