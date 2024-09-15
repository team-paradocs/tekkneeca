cmake_minimum_required(VERSION 3.10)
project(convex_plane_decomposition_ros)

# Catkin dependencies
set(dependencies
	rclcpp
	grid_map_core
	grid_map_ros
	grid_map_cv
	grid_map_msgs
	geometry_msgs
	convex_plane_decomposition
	convex_plane_decomposition_msgs
	tf2_ros
	visualization_msgs
	OpenCV
)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(grid_map_cv REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(convex_plane_decomposition REQUIRED)
find_package(convex_plane_decomposition_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

# OpenCv
find_package(OpenCV REQUIRED)

# Eigen
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

find_package(Boost REQUIRED COMPONENTS
  system
  filesystem
)

# Cpp standard version
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

###########
## Build ##
###########

include_directories(
	include
	${EIGEN3_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
	src/ConvexPlaneDecompositionRos.cpp
	src/MessageConversion.cpp
	src/ParameterLoading.cpp
	src/RosVisualizations.cpp
	)
ament_target_dependencies(${PROJECT_NAME}
	${dependencies}
)

add_executable(${PROJECT_NAME}_node
	src/ConvexPlaneDecompositionNode.cpp
)
ament_target_dependencies(${PROJECT_NAME}_node
  ${dependencies}
)
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
)

add_executable(${PROJECT_NAME}_add_noise
	src/noiseNode.cpp
)
ament_target_dependencies(${PROJECT_NAME}_add_noise
  ${dependencies}
)
target_link_libraries(${PROJECT_NAME}_add_noise
  ${PROJECT_NAME}
)

add_executable(${PROJECT_NAME}_save_elevationmap
	src/SaveElevationMapAsImageNode.cpp
	)
ament_target_dependencies(${PROJECT_NAME}_save_elevationmap
	${dependencies}
)
target_link_libraries(${PROJECT_NAME}_save_elevationmap
	${PROJECT_NAME}
)

add_executable(${PROJECT_NAME}_approximation_demo_node
	src/ConvexApproximationDemoNode.cpp
	)
ament_target_dependencies(${PROJECT_NAME}_approximation_demo_node
	${dependencies}
)
target_link_libraries(${PROJECT_NAME}_approximation_demo_node
	${PROJECT_NAME}
)

#############
## Install ##
#############

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

install(DIRECTORY config data launch rviz
	DESTINATION share/${PROJECT_NAME}
)

#############
## Testing ##
#############
add_executable(${PROJECT_NAME}_TestShapeGrowing
	test/TestShapeGrowing.cpp
)
ament_target_dependencies(${PROJECT_NAME}_TestShapeGrowing
	${dependencies}
)
target_link_libraries(${PROJECT_NAME}_TestShapeGrowing
	${PROJECT_NAME}
)

install(TARGETS 
  ${PROJECT_NAME}_node
  ${PROJECT_NAME}_add_noise
  ${PROJECT_NAME}_save_elevationmap
  ${PROJECT_NAME}_approximation_demo_node
  ${PROJECT_NAME}_TestShapeGrowing
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(${dependencies})  
ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_package()