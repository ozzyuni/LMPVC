cmake_minimum_required(VERSION 3.8)
project(lmpvc_controller)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(lmpvc_interfaces REQUIRED)
find_package(std_srvs REQUIRED)
find_package(lmpvc_gripper REQUIRED)
find_package(pluginlib REQUIRED)

include_directories(include)
add_executable(lmpvc_controller src/lmpvc_controller.cpp)
add_library(moveit_controller STATIC src/moveit_controller.cpp)
add_library(moveit_services STATIC src/moveit_services.cpp)
add_library(limit_cartesian_speed STATIC src/limit_cartesian_speed.cpp)

ament_target_dependencies(
  limit_cartesian_speed
  moveit_core
  rclcpp
)

ament_target_dependencies(
  moveit_controller
  geometry_msgs
  moveit_core
  moveit_ros_planning_interface
  rclcpp
  std_srvs
)

ament_target_dependencies(
  moveit_services
  geometry_msgs
  moveit_core
  moveit_ros_planning_interface
  lmpvc_interfaces
  rclcpp
)

target_link_libraries(lmpvc_controller moveit_controller moveit_services limit_cartesian_speed)

ament_target_dependencies(
  lmpvc_controller
  geometry_msgs
  moveit_ros_planning_interface
  rclcpp
  lmpvc_gripper
)

install(TARGETS
  moveit_controller
  moveit_services
  limit_cartesian_speed
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS
  lmpvc_controller
  DESTINATION lib/${PROJECT_NAME})

# Install launch files.
install(DIRECTORY
launch
DESTINATION share/${PROJECT_NAME}/
)

ament_package()
