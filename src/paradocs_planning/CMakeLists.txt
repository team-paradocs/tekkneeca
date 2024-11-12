cmake_minimum_required(VERSION 3.8)
project(paradocs_planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(graph_msgs REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

set(LIBRARIES
  # Components
  moveit_global_planner_component
  moveit_hybrid_planning_manager
  moveit_local_planner_component
  # Plugins
  forward_trajectory_plugin
  motion_planning_pipeline_plugin
  simple_sampler_plugin
  motion_compensation_plugin
)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  moveit_core
  moveit_msgs
  moveit_ros_planning
  moveit_ros_planning_interface
  pluginlib
  rclcpp
  rclcpp_action
  rclcpp_components
  std_msgs
  graph_msgs
  moveit_visual_tools
  std_srvs
  control_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  trajectory_msgs
  shape_msgs
)

set(THIS_PACKAGE_INCLUDE_DIRS
  src/global_planner/global_planner_component/include/
  src/global_planner/global_planner_plugins/include/
  src/hybrid_planning_manager/hybrid_planning_manager_component/include/
  src/hybrid_planning_manager/planner_logic_plugins/include/
  src/local_planner/local_planner_component/include/
  src/local_planner/trajectory_operator_plugins/include/
  src/local_planner/local_constraint_solver_plugins/include/
)

# Include Cpp "include" directory
include_directories(${THIS_PACKAGE_INCLUDE_DIRS})
include_directories(include)

add_subdirectory(src/hybrid_planning_manager)
add_subdirectory(src/global_planner)
add_subdirectory(src/local_planner)
add_subdirectory(src/test)

rclcpp_components_register_nodes(moveit_hybrid_planning_manager "moveit::hybrid_planning::HybridPlanningManager")
rclcpp_components_register_nodes(moveit_global_planner_component "moveit::hybrid_planning::GlobalPlannerComponent")
rclcpp_components_register_nodes(moveit_local_planner_component "moveit::hybrid_planning::LocalPlannerComponent")

install(TARGETS ${LIBRARIES}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)

# install(TARGETS cancel_action hybrid_planning_demo_node
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION lib/${PROJECT_NAME}
#   INCLUDES DESTINATION include)

install(DIRECTORY ${THIS_PACKAGE_INCLUDE_DIRS} DESTINATION include)

# install(DIRECTORY test/launch DESTINATION share/${PROJECT_NAME})
# install(DIRECTORY test/config DESTINATION share/${PROJECT_NAME})

pluginlib_export_plugin_description_file(paradocs_planning plugins/moveit_planning_pipeline_plugin.xml)
pluginlib_export_plugin_description_file(paradocs_planning plugins/simple_sampler_plugin.xml)
pluginlib_export_plugin_description_file(paradocs_planning plugins/motion_compensation_plugin.xml)
pluginlib_export_plugin_description_file(paradocs_planning plugins/forward_trajectory_plugin.xml)

ament_export_include_directories(include)
ament_export_libraries(${LIBRARIES})
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

# Create Cpp executable
add_executable(talker src/publisher_member_function.cpp)
ament_target_dependencies(talker rclcpp std_msgs)
add_executable(listener src/subscriber_member_function.cpp)
ament_target_dependencies(listener rclcpp std_msgs)
add_executable(hybrid_planning_demo src/test/hybrid_planning_demo_node.cpp)
ament_target_dependencies(hybrid_planning_demo rclcpp std_msgs moveit_core moveit_ros_planning_interface moveit_msgs)
add_executable(continuous_motion_demo src/test/continuous_motion_demo.cpp)
ament_target_dependencies(continuous_motion_demo rclcpp std_msgs moveit_core moveit_ros_planning_interface moveit_msgs)
add_executable(robot_motion_planning src/robot_motion_planning.cpp)
ament_target_dependencies(robot_motion_planning rclcpp std_msgs control_msgs moveit_msgs moveit_core moveit_ros_planning_interface)

# Install Cpp executables
install(TARGETS
  talker
  listener
  hybrid_planning_demo
  continuous_motion_demo
  robot_motion_planning
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/publisher_member_function.py
  scripts/subscriber_member_function.py
  scripts/rosbag_tf_broadcaster.py
  scripts/continuous_pose_publisher.py
  scripts/drill_pose_publisher.py
  scripts/realistic_pose_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install all launch file
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

# For testing purpose, not important
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

ament_package()
