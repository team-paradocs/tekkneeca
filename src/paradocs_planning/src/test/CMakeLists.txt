add_executable(cancel_action cancel_action.cpp)
ament_target_dependencies(cancel_action ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(cancel_action ${LIBRARIES})

add_executable(hybrid_planning_demo_node hybrid_planning_demo_node.cpp)
ament_target_dependencies(hybrid_planning_demo_node ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(hybrid_planning_demo_node ${LIBRARIES})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(Boost REQUIRED COMPONENTS filesystem)

  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_flake8_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
endif()
