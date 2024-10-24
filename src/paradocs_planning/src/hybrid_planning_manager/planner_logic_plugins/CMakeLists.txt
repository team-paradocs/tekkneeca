add_library(motion_compensation_plugin SHARED
  src/motion_compensation.cpp
)
set_target_properties(motion_compensation_plugin PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
ament_target_dependencies(motion_compensation_plugin ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(motion_compensation_plugin moveit_hybrid_planning_manager)
