cmake_minimum_required(VERSION 3.8)
project(autonav_state)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(autonav_msgs REQUIRED)
find_package(scr_core REQUIRED)

add_executable(statemanager src/systemstate.cpp)
ament_target_dependencies(statemanager rclcpp autonav_msgs scr_core)

install(TARGETS
  statemanager
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
