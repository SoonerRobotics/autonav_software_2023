cmake_minimum_required(VERSION 3.8)
project(autonav_libs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(autonav_msgs REQUIRED)

# C++ Library Garbage
include_directories(include)
add_library(autonav_libs src/autonav.cpp)
ament_export_targets(autonav_libs HAS_LIBRARY_TARGET)
ament_target_dependencies(autonav_libs
  "rclcpp"
  "autonav_msgs"
)
install(
  DIRECTORY include/autonav_libs
  DESTINATION include
)

install(
  TARGETS autonav_libs
  EXPORT autonav_libs
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Install src/canbus.py and src/conbus.py
install(
  PROGRAMS src/canbus.py src/conbus.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()