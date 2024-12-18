cmake_minimum_required(VERSION 3.8)
project(micpp_manymove_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(action_msgs REQUIRED)
find_package(control_msgs REQUIRED)

# Generate messages and actions
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/MoveManipulator.action"
  "action/MoveManipulatorSequence.action"
  "msg/MovementConfig.msg"
  "msg/MoveManipulatorGoal.msg"
  DEPENDENCIES geometry_msgs action_msgs
)

# Ensure that message generation is complete before proceeding
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

ament_export_dependencies(rosidl_default_runtime)

include_directories(include)

# Create the shared library
add_library(micpp_manymove_planner_lib
  src/micpp_manymove_planner.cpp
)

# Link dependencies to the shared library
ament_target_dependencies(micpp_manymove_planner_lib
  rclcpp
  moveit_core
  moveit_ros_planning_interface
  tf2_geometry_msgs
  geometry_msgs
  moveit_visual_tools
  pluginlib
  rosidl_default_runtime
)

# Link the typesupport library
target_link_libraries(micpp_manymove_planner_lib
  ${cpp_typesupport_target}
)

# Install the shared library
install(TARGETS
  micpp_manymove_planner_lib
  DESTINATION lib
)

# Main example node
add_executable(micpp_manymove_planner_node src/main.cpp)
target_link_libraries(micpp_manymove_planner_node
  micpp_manymove_planner_lib
  ${cpp_typesupport_target}
)
ament_target_dependencies(micpp_manymove_planner_node
  rclcpp
  moveit_core
  moveit_ros_planning_interface
  tf2_geometry_msgs
  geometry_msgs
  moveit_visual_tools
)

install(TARGETS micpp_manymove_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

# Move Action Server (Single Action)
add_executable(action_server_node
  src/action_server_node.cpp
  src/action_server.cpp
)
target_link_libraries(action_server_node
  micpp_manymove_planner_lib
  ${cpp_typesupport_target}
)
ament_target_dependencies(action_server_node
  rclcpp
  rclcpp_action
  moveit_core
  moveit_ros_planning_interface
  tf2_geometry_msgs
  geometry_msgs
  moveit_visual_tools
  action_msgs
)

install(TARGETS action_server_node
  DESTINATION lib/${PROJECT_NAME}
)

# Move Client Example
add_executable(client_example_node src/client_example_node.cpp)
target_link_libraries(client_example_node
  micpp_manymove_planner_lib
  ${cpp_typesupport_target}
)
ament_target_dependencies(client_example_node
  rclcpp
  rclcpp_action
  geometry_msgs
  action_msgs
)

install(TARGETS client_example_node
  DESTINATION lib/${PROJECT_NAME}
)

# Standalone MoveItCpp Demo
add_executable(standalone_micpp_demo
  src/standalone_micpp_demo.cpp
)

# Link the new executable to the shared library and necessary dependencies
target_link_libraries(standalone_micpp_demo
  micpp_manymove_planner_lib
  ${cpp_typesupport_target}
)
ament_target_dependencies(standalone_micpp_demo
  rclcpp
  moveit_core
  moveit_ros_planning_interface
  tf2_geometry_msgs
  geometry_msgs
  moveit_visual_tools
)

# Install the new executable
install(TARGETS standalone_micpp_demo
  DESTINATION lib/${PROJECT_NAME}
)

# Install config & launch files
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY include/
  DESTINATION include/
)

# Exporting
ament_export_dependencies(
  rclcpp
  moveit_core
  moveit_ros_planning_interface
  tf2_geometry_msgs
  geometry_msgs
  moveit_visual_tools
  action_msgs
  rclcpp_action
  rosidl_default_runtime
)
ament_export_libraries(micpp_manymove_planner_lib)
ament_export_include_directories(include)

ament_package()
