cmake_minimum_required(VERSION 3.8)
project(smart_diffbot_navigation)

# Default to C99
set(CMAKE_C_STANDARD 99)


# Default to C++14
set(CMAKE_CXX_STANDARD 14)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav2_common REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(nav2_behaviors REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(std_srvs REQUIRED)
find_package(smart_diffbot_msgs REQUIRED)
find_package(ros2_aruco_interfaces REQUIRED)

include_directories(
  include
)

# Dependencies
set(dependencies
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  std_msgs
  visualization_msgs
  nav2_util
  nav2_msgs
  nav_msgs
  geometry_msgs
  builtin_interfaces
  tf2_ros
  nav2_costmap_2d
  nav2_core
  pluginlib
  rclcpp_components
  nav2_behavior_tree
  nav2_behaviors
  behaviortree_cpp_v3
  std_srvs
  smart_diffbot_msgs
  ros2_aruco_interfaces
)

install(DIRECTORY
  include
  launch
  config
  behavior_trees
  DESTINATION 
  share/${PROJECT_NAME}/
)

# Plugins (planners, controllers, behaviors)
set(plugin_library_name ${PROJECT_NAME}_plugin)

add_library(${plugin_library_name} SHARED
  src/planner_plugins/straight_line_planner.cpp
  src/behavior_plugins/approach_dock.cpp
  src/behavior_plugins/dock.cpp
   
  src/bt_plugins/dock_action.cpp 
)

ament_target_dependencies(${plugin_library_name}
  ${dependencies}
)

target_compile_definitions(${plugin_library_name} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(nav2_core planner_plugins.xml)
pluginlib_export_plugin_description_file(nav2_core behavior_plugins.xml)

install(TARGETS ${plugin_library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(FILES planner_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

install(FILES behavior_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)


# BT plugins 
add_library(smart_diffbot_navigation_approach_dock_action_bt_node SHARED src/bt_plugins/approach_dock_action.cpp)
list(APPEND bt_plugin_libs smart_diffbot_navigation_approach_dock_action_bt_node)

add_library(smart_diffbot_navigation_dock_action_bt_node SHARED src/bt_plugins/dock_action.cpp)
list(APPEND bt_plugin_libs smart_diffbot_navigation_dock_action_bt_node)

foreach(bt_plugin ${bt_plugin_libs})
  ament_target_dependencies(${bt_plugin} ${dependencies})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

install(TARGETS ${bt_plugin_libs}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(FILES bt_plugins.xml DESTINATION share/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${plugin_library_name})
ament_export_libraries(${bt_plugin_libs})
ament_export_dependencies(${dependencies})
ament_package()