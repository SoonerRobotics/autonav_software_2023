cmake_minimum_required(VERSION 3.8)
project(autonav_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

# generate messages
set(msg_files
  "msg/MotorFeedback.msg"
  "msg/MotorInput.msg"
  "msg/IMUData.msg"
  "msg/SteamInput.msg"
  "msg/GPSFeedback.msg"
  "msg/Position.msg"
  "msg/GoalPoint.msg"
  "msg/Waypoint.msg"
  "msg/ObjectDetection.msg"
  "msg/Path.msg"
  "msg/Obstacle.msg"
  "msg/Obstacles.msg"
  "msg/MotorControllerDebug.msg"
  "msg/SafetyLights.msg"
  "msg/Conbus.msg"
  "msg/PathingDebug.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()