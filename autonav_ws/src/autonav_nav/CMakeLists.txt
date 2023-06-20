cmake_minimum_required(VERSION 3.8)
project(autonav_nav)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
	src/path_resolver.py
	src/astar.py
	src/pure_pursuit.py
	DESTINATION lib/${PROJECT_NAME}
)

ament_package()