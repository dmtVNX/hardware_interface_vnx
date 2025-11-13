cmake_minimum_required(VERSION 3.8)
project(motor_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(can_msgs REQUIRED)
find_package(motor_feedback_msg REQUIRED)

# Include directories
include_directories(include)

# ========== EXISTING LIBRARIES ==========
add_library(motor_config
  src/motor_config.cpp
  src/robstride.cpp
)
ament_target_dependencies(motor_config
  rclcpp
  std_msgs
  can_msgs
  motor_feedback_msg
)
target_link_libraries(motor_config)

# ========== EXISTING NODE (3 motors) ==========
add_executable(motor_control_node
  src/motor_control_node.cpp
)
target_link_libraries(motor_control_node motor_config)
ament_target_dependencies(motor_control_node
  rclcpp
  std_msgs
  can_msgs
  motor_feedback_msg
)

# ========== NEW: QUADRUPED CONTROLLER ==========
add_library(quadruped_controller
  src/quadruped_controller.cpp
)
target_link_libraries(quadruped_controller motor_config)
ament_target_dependencies(quadruped_controller
  rclcpp
  std_msgs
  can_msgs
  motor_feedback_msg
)

# ========== NEW: QUADRUPED NODE (12 motors) ==========
add_executable(quadruped_control_node
  src/quadruped_control_node.cpp
)
target_link_libraries(quadruped_control_node 
  quadruped_controller
  motor_config
)
ament_target_dependencies(quadruped_control_node
  rclcpp
  std_msgs
  can_msgs
  motor_feedback_msg
)

# ========== INSTALL ==========
install(TARGETS motor_config quadruped_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


install(TARGETS motor_control_node quadruped_control_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch scripts
  DESTINATION share/${PROJECT_NAME}/
  USE_SOURCE_PERMISSIONS
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(motor_config quadruped_controller)
ament_export_dependencies(
  rclcpp
  std_msgs
  can_msgs
  motor_feedback_msg
)

ament_package()