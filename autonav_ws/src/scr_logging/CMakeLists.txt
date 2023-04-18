cmake_minimum_required(VERSION 3.8)
project(scr_logging)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(scr_msgs REQUIRED)
find_package(scr_core REQUIRED)

add_executable(logging src/logging.cpp)
ament_target_dependencies(logging rclcpp std_msgs scr_msgs scr_core)

install(TARGETS
  logging
  DESTINATION lib/${PROJECT_NAME})

ament_package()
