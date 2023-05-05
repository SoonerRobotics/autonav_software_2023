cmake_minimum_required(VERSION 3.8)
project(autonav_manual)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(autonav_msgs REQUIRED)
find_package(scr_core REQUIRED)

# C++
add_executable(remote src/remote.cpp)
ament_target_dependencies(remote rclcpp sensor_msgs autonav_msgs scr_core)

add_executable(steamremote src/remote_steamcontroller.cpp)
ament_target_dependencies(steamremote rclcpp sensor_msgs autonav_msgs scr_core)

install(TARGETS
  remote
  steamremote
  DESTINATION lib/${PROJECT_NAME}
)

# Python
install(PROGRAMS
  src/steam.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()