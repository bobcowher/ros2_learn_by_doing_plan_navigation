cmake_minimum_required(VERSION 3.8)
project(bumperbot_firmware_v2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

# Include directories
include_directories(include)

# Create the robot controller library
add_library(robot_controller src/robot_controller.cpp)
target_include_directories(robot_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Create the main executable
add_executable(robot_driver_node src/robot_driver_node.cpp)
target_link_libraries(robot_driver_node robot_controller)

# ROS2 dependencies
ament_target_dependencies(robot_driver_node
  rclcpp
  geometry_msgs
)

# Install targets
install(TARGETS
  robot_driver_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
