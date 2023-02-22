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
find_package(autonav_libs REQUIRED)

# C++
add_executable(example src/example.cpp)
ament_target_dependencies(example rclcpp sensor_msgs autonav_msgs autonav_libs)

install(TARGETS
  example
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()