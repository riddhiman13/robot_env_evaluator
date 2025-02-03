cmake_minimum_required(VERSION 3.12)
project(robot_env_evaluator)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# set the options
if(${CMAKE_CURRENT_SOURCE_DIR} STREQUAL ${CMAKE_SOURCE_DIR})
    set(ROENVEVAL_BUILD_EXAMPLE_DEFAULT ON)
else()
    set(ROENVEVAL_BUILD_EXAMPLE_DEFAULT OFF)
endif()

if("${CMAKE_BUILD_TYPE}" STREQUAL "Debug")
    set(ROENVEVAL_DEBUG_MODE_DEFAULT ON)
else()
    set(ROENVEVAL_DEBUG_MODE_DEFAULT OFF)
endif()

option(ROENVEVAL_BUILD_EXAMPLE "Build the example" ${ROENVEVAL_BUILD_EXAMPLE_DEFAULT})
option(ROENVEVAL_DEBUG_MODE "Build with debug mode" ${ROENVEVAL_DEBUG_MODE_DEFAULT})

# display the options
message(STATUS 
  "[robot_env_evaluator] Options:
         ROENVEVAL_BUILD_EXAMPLE:     ${ROENVEVAL_BUILD_EXAMPLE}
            ROENVEVAL_DEBUG_MODE:     ${ROENVEVAL_DEBUG_MODE}
")

## System dependencies are found with CMake's conventions
find_package(pinocchio REQUIRED)

## Specify headers to define local paths
set(ROENVEVAL_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR})
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/include/robot_env_evaluator/robot_env_evaluator_path.h.in
  ${CMAKE_CURRENT_BINARY_DIR}/include/robot_env_evaluator/robot_env_evaluator_path.h
)

## Declare a C++ library
add_library(robot_env_evaluator
  src/robot_env_evaluator.cpp
)
target_include_directories(robot_env_evaluator 
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
  PUBLIC ${CMAKE_CURRENT_BINARY_DIR}/include
)
target_link_libraries(robot_env_evaluator
  pinocchio::pinocchio
)