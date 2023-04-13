cmake_minimum_required(VERSION 3.8)
project(autonav_pathing)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(autonav_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# messages and services
# include cpp include directory
include_directories(include)

# Create cpp executables

# install cpp executables

# install python programs
install(PROGRAMS
  src/pure_pursuit/pure_pursuit.py
  src/pure_pursuit/lookahead_finder.py
  src/pure_pursuit/pursuit_test.py
  src/pure_pursuit/random_path.py
  src/path_planning/path_planning.py
  src/path_planning/tangent_based.py
  src/path_planning/planning_test.py
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
