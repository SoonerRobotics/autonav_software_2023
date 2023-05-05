cmake_minimum_required(VERSION 3.8)
project(scr_configuration)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(scr_msgs REQUIRED)
find_package(scr_core REQUIRED)

add_executable(config src/config.cpp)
ament_target_dependencies(config rclcpp std_msgs scr_msgs scr_core)

install(TARGETS
config
  DESTINATION lib/${PROJECT_NAME})

ament_package()
