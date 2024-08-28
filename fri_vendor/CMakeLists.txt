cmake_minimum_required(VERSION 3.18)
project(fri_vendor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(FRI_CLIENT_VERSION 1.15 CACHE STRING "FRI version to use.")

include(ExternalProject)

# find dependencies
find_package(ament_cmake REQUIRED)

# add external project
ExternalProject_Add(FRIClient
  GIT_REPOSITORY https://github.com/KCL-BMEIS/fri.git
  GIT_TAG fri-${FRI_CLIENT_VERSION}
  CMAKE_ARGS
    -DCMAKE_INSTALL_PREFIX=${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}_install
  INSTALL_DIR ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}_install
)

install(
  DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}_install/
  DESTINATION ${CMAKE_INSTALL_PREFIX}
)

ament_package()
