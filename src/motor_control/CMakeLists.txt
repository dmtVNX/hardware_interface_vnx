cmake_minimum_required(VERSION 3.8)
project(motor_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# -----------------------------
# Dependencies
# -----------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(can_msgs REQUIRED)
find_package(motor_feedback_msg REQUIRED)

# ros2_control plugin deps
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

# -----------------------------
# Includes
# -----------------------------
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# -----------------------------
# Driver library (your existing code)
# -----------------------------
add_library(${PROJECT_NAME}_lib SHARED
  src/motor_config.cpp
  src/robstride.cpp
)

ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp std_msgs can_msgs motor_feedback_msg
)

# -----------------------------
# ros2_control hardware plugin (POSITION only)
# -----------------------------

add_library(robstride_hardware_simple SHARED
  src/robstride_system_simple.cpp
)
target_include_directories(robstride_hardware_simple PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

ament_target_dependencies(robstride_hardware_simple
  rclcpp
  can_msgs
  hardware_interface
  pluginlib
)

target_link_libraries(robstride_hardware_simple
  ${PROJECT_NAME}_lib
)

pluginlib_export_plugin_description_file(hardware_interface robstride_hardware_simple_plugin.xml)

# -----------------------------
# Executable
# -----------------------------
add_executable(motor_control_node src/motor_control_node.cpp)
target_link_libraries(motor_control_node ${PROJECT_NAME}_lib)
ament_target_dependencies(motor_control_node
  rclcpp std_msgs can_msgs motor_feedback_msg
)

# -----------------------------
# Install
# -----------------------------
# Libraries & executables
install(TARGETS
  ${PROJECT_NAME}_lib
  robstride_hardware_simple
  motor_control_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Headers
install(
  DIRECTORY include/
  DESTINATION include/
)

# Plugin XML
install(
  FILES robstride_hardware_simple_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

# Config & Launch
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# -----------------------------
# Tests
# -----------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
