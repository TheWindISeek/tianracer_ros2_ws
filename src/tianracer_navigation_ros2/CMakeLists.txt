cmake_minimum_required(VERSION 3.8)
project(tianracer_navigation_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# C++17 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 设置符号可见性，确保 BT_RegisterNodesFromPlugin 被导出
set(CMAKE_CXX_VISIBILITY_PRESET default)
set(CMAKE_VISIBILITY_INLINES_HIDDEN OFF)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(nav2_behavior_tree REQUIRED)

# Include directories
include_directories(include)

set(dependencies
  rclcpp
  rclcpp_action
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  behaviortree_cpp_v3
  nav2_behavior_tree
)

# ============================================================================
# 行为树插件库
# ============================================================================

# IsGoalInFront 条件节点
add_library(tianracer_is_goal_in_front_bt_node SHARED
  src/plugins/condition/is_goal_in_front_condition.cpp
)
target_compile_definitions(tianracer_is_goal_in_front_bt_node PRIVATE BT_PLUGIN_EXPORT)
ament_target_dependencies(tianracer_is_goal_in_front_bt_node ${dependencies})

# ShuttleToOrientation 动作节点
add_library(tianracer_shuttle_to_orientation_bt_node SHARED
  src/plugins/action/shuttle_to_orientation_action.cpp
)
target_compile_definitions(tianracer_shuttle_to_orientation_bt_node PRIVATE BT_PLUGIN_EXPORT)
ament_target_dependencies(tianracer_shuttle_to_orientation_bt_node ${dependencies})

# ============================================================================
# 安装
# ============================================================================

# Install libraries
install(TARGETS
  tianracer_is_goal_in_front_bt_node
  tianracer_shuttle_to_orientation_bt_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install include files
install(DIRECTORY include/
  DESTINATION include
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install param files (Nav2 configuration)
install(DIRECTORY param/
  DESTINATION share/${PROJECT_NAME}/param
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install behavior trees
install(DIRECTORY behavior_trees/
  DESTINATION share/${PROJECT_NAME}/behavior_trees
)

# Install docs
install(DIRECTORY docs/
  DESTINATION share/${PROJECT_NAME}/docs
)

# Export dependencies
ament_export_include_directories(include)
ament_export_libraries(
  tianracer_is_goal_in_front_bt_node
  tianracer_shuttle_to_orientation_bt_node
)
ament_export_dependencies(${dependencies})

ament_package()
