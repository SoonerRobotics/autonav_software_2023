cmake_minimum_required(VERSION 3.8)
project(autonav_vision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(autonav_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(scr_core REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)

# Install Python executables
install(PROGRAMS
  src/transformations.py
  DESTINATION lib/${PROJECT_NAME}
)
 
# Install Python programs
install(PROGRAMS
  src/circumscriber.py
  src/transformations.py
  DESTINATION lib/${PROJECT_NAME}
)

# C++
add_executable(expandify src/expandify.cpp)
ament_target_dependencies(expandify rclcpp sensor_msgs autonav_msgs scr_core nav_msgs cv_bridge image_transport)

install(TARGETS
  expandify
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()
ament_package()