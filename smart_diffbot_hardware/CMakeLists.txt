cmake_minimum_required(VERSION 3.8)
project(smart_diffbot_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# Complile library
add_library(
  ${PROJECT_NAME} 
  SHARED
  src/dc_motor_velocity_interface.cpp
) 
target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  include
)
target_link_libraries(
  ${PROJECT_NAME} 
  # libraries to link with here
)

ament_target_dependencies(
  ${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface hardware_plugins.xml)

# Install
install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)
install(
  DIRECTORY include/
  DESTINATION include
)

# Export
ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# Testing
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

ament_package()

