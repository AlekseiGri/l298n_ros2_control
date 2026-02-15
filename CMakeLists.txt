cmake_minimum_required(VERSION 3.8)
project(l298n_ros2_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_interface REQUIRED)
# find_package(ros2_control REQUIRED)

# Build shared library (plugin)
add_library(l298n_hardware SHARED
  src/l298n_system.cpp
)

target_include_directories(l298n_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(l298n_hardware
  rclcpp
  pluginlib
  hardware_interface
  controller_interface
  # ros2_control
)

# Export plugin description to pluginlib
pluginlib_export_plugin_description_file(
  hardware_interface
  plugin_description.xml
)

# Install library and headers
install(
  TARGETS l298n_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

# Lint / tests (optional)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
