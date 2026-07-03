cmake_minimum_required(VERSION 3.24)
project(
  strada
  VERSION 0.1.0
  LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Suppress clang-tidy checks in the build/binary directory
file(WRITE "${CMAKE_BINARY_DIR}/.clang-tidy" "Checks: '-*'\n")

option(STRADA_BUILD_VIS "Build the Strada 2D Vis library and application" ${PROJECT_IS_TOP_LEVEL})
option(STRADA_BUILD_TESTS "Build tests for the Strada project" ${PROJECT_IS_TOP_LEVEL})

add_subdirectory(external)
add_subdirectory(src)
add_subdirectory(examples)

if(STRADA_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()
