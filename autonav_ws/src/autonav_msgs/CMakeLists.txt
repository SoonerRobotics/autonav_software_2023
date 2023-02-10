cmake_minimum_required(VERSION 3.5)
project(autonav_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
	"msg/MotorFeedback.msg"
	"msg/MotorInput.msg"
	"msg/ImuData.msg"
	"msg/GPSFeedback.msg"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()