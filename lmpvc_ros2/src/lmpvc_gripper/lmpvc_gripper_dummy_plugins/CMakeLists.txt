cmake_minimum_required(VERSION 3.8)
project(lmpvc_gripper_dummy_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(pluginlib REQUIRED)
find_package(lmpvc_gripper REQUIRED)
find_package(lmpvc_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)

# Generate plugins
pluginlib_export_plugin_description_file(lmpvc_gripper plugins.xml)

include_directories(include)

add_library(dummy_gripper src/dummy_gripper.cpp)
target_compile_features(dummy_gripper PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(dummy_gripper PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(
  dummy_gripper
  rclcpp
  pluginlib
  lmpvc_gripper
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS dummy_gripper
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
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

ament_export_include_directories(
  include
)
ament_export_libraries(
  dummy_gripper
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
