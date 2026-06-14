cmake_minimum_required(VERSION 3.24)
project(
  strada
  VERSION 0.1.0
  LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Dependencies via FetchContent
include(FetchContent)

# 1. pugixml
FetchContent_Declare(
  pugixml
  URL https://github.com/zeux/pugixml/releases/download/v1.14/pugixml-1.14.tar.gz
      FIND_PACKAGE_ARGS)

# 1. Eigen
FetchContent_Declare(
  eigen URL https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz
            FIND_PACKAGE_ARGS NAMES Eigen3)

# 1. GoogleTest
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.tar.gz
      FIND_PACKAGE_ARGS NAMES GTest)

set(EIGEN_BUILD_TESTING
    OFF
    CACHE BOOL "" FORCE)
set(gtest_force_shared_crt
    ON
    CACHE BOOL "" FORCE)

FetchContent_MakeAvailable(pugixml eigen googletest)

# Strada Library
add_library(strada src/parser/parser.cpp)

target_include_directories(
  strada PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                $<INSTALL_INTERFACE:include>)

target_link_libraries(strada PRIVATE pugixml)

# Tests
enable_testing()

add_executable(test_strada tests/ast/test_parser.cpp)

target_compile_definitions(test_strada PRIVATE STRADA_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/tests/data")

target_link_libraries(test_strada PRIVATE strada GTest::gtest_main)

# Enable CTest
include(GoogleTest)
gtest_discover_tests(test_strada)
