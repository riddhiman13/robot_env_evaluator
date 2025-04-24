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
option(ROENVEVAL_BUILD_TESTS "Build the unit tests" ${ROENVEVAL_BUILD_EXAMPLE_DEFAULT})

# display the options
message(STATUS 
  "[robot_env_evaluator] Options:
         ROENVEVAL_BUILD_EXAMPLE:     ${ROENVEVAL_BUILD_EXAMPLE}
            ROENVEVAL_DEBUG_MODE:     ${ROENVEVAL_DEBUG_MODE}
")

## System dependencies are found with CMake's conventions
find_package(pinocchio REQUIRED)
target_compile_options(pinocchio::pinocchio INTERFACE -Wno-cpp)

## Specify headers to define local paths
set(ROENVEVAL_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR})
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/include/robot_env_evaluator/robot_env_evaluator_path.h.in
  ${CMAKE_CURRENT_BINARY_DIR}/include/robot_env_evaluator/robot_env_evaluator_path.h
)

## Declare a C++ library
add_library(robot_env_evaluator SHARED
  ${CMAKE_CURRENT_SOURCE_DIR}/src/robot_env_evaluator.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/robot_presets.cpp
)
target_include_directories(robot_env_evaluator PUBLIC 
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_BINARY_DIR}/include
)
target_link_libraries(robot_env_evaluator PUBLIC
  pinocchio::pinocchio
)
if(ROENVEVAL_DEBUG_MODE)
    target_compile_definitions(robot_env_evaluator PRIVATE ROENVEVAL_DEBUG)
    target_compile_options(robot_env_evaluator PRIVATE -g)
endif()

## Declare the example
if(ROENVEVAL_BUILD_EXAMPLE)
  list(APPEND ROENVEVAL_EXAMPLE
    ${CMAKE_CURRENT_SOURCE_DIR}/example/robot_env_evaluator_example.cpp
  )

  foreach(EXAMPLE_FILE ${ROENVEVAL_EXAMPLE})
    get_filename_component(EXAMPLE_NAME ${EXAMPLE_FILE} NAME_WE)
    add_executable(${EXAMPLE_NAME} ${EXAMPLE_FILE})
    target_link_libraries(${EXAMPLE_NAME} PUBLIC robot_env_evaluator)
    if(ROENVEVAL_DEBUG_MODE)
      target_compile_options(${EXAMPLE_NAME} PRIVATE -g)
    endif()
  endforeach()
endif()

## Declare the tests
if(ROENVEVAL_BUILD_TESTS)
  enable_testing()
  find_package(GTest REQUIRED)

  add_executable(test_robot_env_evaluator
    ${CMAKE_CURRENT_SOURCE_DIR}/test/test_robot_env_evaluator.cpp
  )
  target_link_libraries(test_robot_env_evaluator PRIVATE
    robot_env_evaluator
    GTest::GTest
    GTest::Main
  )
  add_test(NAME RobotEnvEvaluatorTests COMMAND test_robot_env_evaluator)
endif()