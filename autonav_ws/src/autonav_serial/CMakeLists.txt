cmake_minimum_required(VERSION 3.8)
project(autonav_serial)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(scr_core REQUIRED)
find_package(scr_msgs REQUIRED)
find_package(autonav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
	src/serial_node.py
	src/imu.py
	src/camera.py
	src/safety_lights.py
	src/jams.py
	DESTINATION lib/${PROJECT_NAME}
)

# Install Vectornav Dependencies
set(VECTORNAV_DIR "/usr/local/vectornav/cpp")
add_library(vectornav SHARED
	${VECTORNAV_DIR}/src/attitude.cpp
	${VECTORNAV_DIR}/src/compositedata.cpp
	${VECTORNAV_DIR}/src/conversions.cpp
	${VECTORNAV_DIR}/src/criticalsection.cpp
	${VECTORNAV_DIR}/src/dllvalidator.cpp
	${VECTORNAV_DIR}/src/error_detection.cpp
	${VECTORNAV_DIR}/src/event.cpp
	${VECTORNAV_DIR}/src/ezasyncdata.cpp
	${VECTORNAV_DIR}/src/memoryport.cpp
	${VECTORNAV_DIR}/src/packet.cpp
	${VECTORNAV_DIR}/src/packetfinder.cpp
	${VECTORNAV_DIR}/src/port.cpp
	${VECTORNAV_DIR}/src/position.cpp
	${VECTORNAV_DIR}/src/rtcmlistener.cpp
	${VECTORNAV_DIR}/src/rtcmmessage.cpp
	${VECTORNAV_DIR}/src/searcher.cpp
	${VECTORNAV_DIR}/src/sensors.cpp
	${VECTORNAV_DIR}/src/serialport.cpp
	${VECTORNAV_DIR}/src/thread.cpp
	${VECTORNAV_DIR}/src/types.cpp
	${VECTORNAV_DIR}/src/util.cpp
	${VECTORNAV_DIR}/src/utilities.cpp
	${VECTORNAV_DIR}/src/vntime.cpp
)
target_include_directories(vectornav PUBLIC
	${VECTORNAV_DIR}/include
)

add_executable(vectornav_node
	src/vectornav.cpp
)
ament_target_dependencies(vectornav_node
	scr_core
	scr_msgs
	autonav_msgs
	rclcpp
	std_msgs
	sensor_msgs
)
target_link_libraries(vectornav_node
	vectornav
)

ament_export_include_directories(include)
ament_export_libraries(vectornav)

install(TARGETS
	vectornav_node
	DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS vectornav EXPORT export_vectornav
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib
	RUNTIME DESTINATION bin
)

ament_package()