cmake_minimum_required(VERSION 2.6)
project(trajectories)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

add_definitions("-Wall")

include_directories(${PROJECT_SOURCE_DIR})

set(library_name ${PROJECT_NAME})

file(GLOB library_source "Trajectory.cpp" "Path.cpp" "*.h")
list(SORT library_source)

add_library(${library_name} SHARED ${library_source})

add_executable(example "Example.cpp")
target_link_libraries(example ${library_name})
