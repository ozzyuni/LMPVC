cmake_minimum_required(VERSION 3.8)
project(lmpvc_gripper)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(lmpvc_interfaces REQUIRED)

include_directories(include)

add_library(gripper_base STATIC src/gripper_base.cpp)
add_executable(gripper_test src/basic_gripper.cpp)
target_include_directories(gripper_test PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(gripper_test PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(
  gripper_base
  lmpvc_interfaces
  rclcpp
)

ament_export_dependencies(lmpvc_interfaces)

target_link_libraries(gripper_test gripper_base)

ament_target_dependencies(
  gripper_test
  pluginlib
  rclcpp
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(
  include
)

ament_export_libraries(
  gripper_base
)

install(TARGETS
  gripper_base
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS
  gripper_test
  DESTINATION lib/${PROJECT_NAME})

ament_package()

