cmake_minimum_required(VERSION 3.8)
project(paradocs_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# find_package(Serial REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(moveit_ros_planning_interface)
find_package(moveit_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)

# Include Cpp "include" directory
include_directories(include)

# Create Cpp executable
add_executable(hello_moveit src/hello_moveit.cpp)
ament_target_dependencies(hello_moveit rclcpp moveit_ros_planning_interface)
add_executable(static_obstacles src/static_obstacles.cpp)
ament_target_dependencies(static_obstacles rclcpp moveit_ros_planning_interface)
# add_executable(drill_bt behavior_tree_code/drill_bt.cpp)
# ament_target_dependencies(drill_bt rclcpp moveit_ros_planning_interface)
add_executable(talker src/publisher_member_function.cpp)
ament_target_dependencies(talker rclcpp std_msgs)
add_executable(listener src/subscriber_member_function.cpp)
ament_target_dependencies(listener rclcpp std_msgs)

add_executable(bt_main behavior_tree_code/bt_main.cpp behavior_tree_code/drill_bt.cpp)
ament_target_dependencies(bt_main rclcpp moveit_ros_planning_interface)


# target_link_libraries(drill_bt behaviortree_cpp)
target_link_libraries(bt_main behaviortree_cpp)


# Install Cpp executables
install(TARGETS
  # point_cloud_saver
  talker
  listener
  hello_moveit
  static_obstacles
  # drill_bt
  bt_main
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/forward_kinematics.py
  scripts/publisher_member_function.py
  scripts/subscriber_member_function.py
  scripts/point_cloud_saver.py
  scripts/open3d_pipeline.py
  scripts/drill_pose_transformer.py
  scripts/aruco_pose_transformer.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install all launch file
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
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

ament_package()
