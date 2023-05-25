cmake_minimum_required(VERSION 3.8)
project(autonav_display)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(autonav_msgs REQUIRED)
find_package(scr_core REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(scr_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED PATHS /usr/local/lib/cmake/opencv4 NO_DEFAULT_PATH)

# IMGUI Dependencies
set(OpenGL_GL_PREFERENCE GLVND)
find_package(glfw3 3.3 REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)
set(IMGUI_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../../deps/imgui)
set(IMGUI_PLOTS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../../deps/implot)
add_library(imgui SHARED
  ${IMGUI_DIR}/imgui.cpp
  ${IMGUI_DIR}/imgui_demo.cpp
  ${IMGUI_DIR}/imgui_draw.cpp
  ${IMGUI_DIR}/imgui_widgets.cpp
  ${IMGUI_DIR}/imgui_tables.cpp
  ${IMGUI_DIR}/backends/imgui_impl_glfw.cpp
  ${IMGUI_DIR}/backends/imgui_impl_opengl3.cpp

  ${IMGUI_PLOTS_DIR}/implot.cpp
  ${IMGUI_PLOTS_DIR}/implot_demo.cpp
  ${IMGUI_PLOTS_DIR}/implot_items.cpp
)
target_include_directories(imgui PUBLIC
  $<BUILD_INTERFACE:${IMGUI_DIR}>
  $<INSTALL_INTERFACE:include>
  $<BUILD_INTERFACE:${IMGUI_DIR}/backends>
  $<INSTALL_INTERFACE:include>
  $<BUILD_INTERFACE:${IMGUI_PLOTS_DIR}>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(imgui PUBLIC glfw OpenGL::GL GLEW::GLEW)
target_compile_definitions(imgui PUBLIC IMGUI_DISABLE_INCLUDE_IMCONFIG_H)
ament_export_targets(export_imgui HAS_LIBRARY_TARGET)

add_executable(primary_display src/display.cpp)
ament_target_dependencies(primary_display
  autonav_msgs
  scr_msgs
  scr_core
  rclcpp
  cv_bridge
  image_transport
  sensor_msgs
  std_msgs
  OpenCV
)
target_link_libraries(primary_display imgui)

ament_export_include_directories(include)
ament_export_libraries(imgui)
ament_export_dependencies(glfw OpenGL GLEW)

# Set OpenCV include directories
include_directories(${OpenCV_INCLUDE_DIRS})

install(FILES
  ${IMGUI_DIR}/imgui.h
  ${IMGUI_DIR}/imgui_internal.h
  ${IMGUI_DIR}/imstb_textedit.h
  ${IMGUI_DIR}/backends/imgui_impl_glfw.h
  ${IMGUI_DIR}/backends/imgui_impl_opengl3.h

  DESTINATION include
)

install(TARGETS imgui EXPORT export_imgui
ARCHIVE DESTINATION lib
LIBRARY DESTINATION lib
RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS primary_display
ARCHIVE DESTINATION lib
LIBRARY DESTINATION lib
RUNTIME DESTINATION lib/${PROJECT_NAME}
)

target_include_directories(primary_display PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
	src/broadcast.py
	DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY ${IMGUI_DIR}/misc/fonts DESTINATION share/${PROJECT_NAME})
install(DIRECTORY fonts DESTINATION share/${PROJECT_NAME})
ament_package()
