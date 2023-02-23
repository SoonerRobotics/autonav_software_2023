cmake_minimum_required(VERSION 3.8)
project(autonav_display)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(autonav_msgs REQUIRED)
find_package(autonav_libs REQUIRED)
find_package(rclcpp REQUIRED)

# IMGUI Dependencies
set(OpenGL_GL_PREFERENCE GLVND)
find_package(glfw3 3.3 REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)
set(IMGUI_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../../imgui)
add_library(imgui SHARED
  ${IMGUI_DIR}/imgui.cpp
  ${IMGUI_DIR}/imgui_demo.cpp
  ${IMGUI_DIR}/imgui_draw.cpp
  ${IMGUI_DIR}/imgui_widgets.cpp
  ${IMGUI_DIR}/imgui_tables.cpp
  ${IMGUI_DIR}/backends/imgui_impl_glfw.cpp
  ${IMGUI_DIR}/backends/imgui_impl_opengl3.cpp
)
target_include_directories(imgui PUBLIC
  $<BUILD_INTERFACE:${IMGUI_DIR}>
  $<INSTALL_INTERFACE:include>
  $<BUILD_INTERFACE:${IMGUI_DIR}/backends>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(imgui PUBLIC glfw OpenGL::GL GLEW::GLEW)
target_compile_definitions(imgui PUBLIC IMGUI_DISABLE_INCLUDE_IMCONFIG_H)
ament_export_targets(export_imgui HAS_LIBRARY_TARGET)

add_executable(imgui_test src/main.cpp)
ament_target_dependencies(imgui_test
  "autonav_msgs"
  "autonav_libs"
  "rclcpp"
)
target_link_libraries(imgui_test imgui)

ament_export_include_directories(include)
ament_export_libraries(imgui)
ament_export_dependencies(glfw OpenGL GLEW)

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

install(TARGETS imgui_test
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
