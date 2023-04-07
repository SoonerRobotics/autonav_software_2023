cmake_minimum_required(VERSION 3.8)
project(my_cpp_py_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# msgs and srvs 
rosidl_generate_interfaces(${PROJECT_NAME}
  # add message types here
  #"path_to_message/message.msg"
)

# Inlcude Cpp "include" directory
include_directories(include)

# Create Cpp executables
#add_executable(executable_name /path_to_executable/executable.cpp)
#ament_target_dependencies(executable_name rclcpp other_dependencies)

# Install Cpp executables
install(TARGETS
  # install executables by name
  #executable_name
  DESTINATION lib/${PROJECT_NAME}
 )
 
 # Use only if not using rosidl_generate_interfaces
 # Install Python modules
 #ament_python_install_package(${PROJECT_NAME})
 
 # Install Python programs
 install(PROGRAMS
   # add programs in format:
   #/path_to_program/program.py
   DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
