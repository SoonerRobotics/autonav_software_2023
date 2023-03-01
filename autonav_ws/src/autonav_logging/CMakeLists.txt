cmake_minimum_required(VERSION 3.8)
project(autonav_logging)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(autonav_msgs REQUIRED)
find_package(autonav_libs REQUIRED)

add_executable(logging src/logging.cpp)
ament_target_dependencies(logging rclcpp std_msgs autonav_msgs autonav_libs)

install(TARGETS
  logging
  DESTINATION lib/${PROJECT_NAME})

ament_package()
