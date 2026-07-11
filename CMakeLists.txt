cmake_minimum_required(VERSION 3.24)

list(PREPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
include(EnsureOutOfSourceBuilds)

project(
  strada
  VERSION 0.1.0
  LANGUAGES CXX)

include(GNUInstallDirs)
set(STRADA_INSTALL_CMAKEDIR "${CMAKE_INSTALL_LIBDIR}/cmake/Strada" CACHE STRING "Directory for CMake package files")

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Suppress clang-tidy checks in the build/binary directory
file(WRITE "${CMAKE_BINARY_DIR}/.clang-tidy" "Checks: '-*'\n")

include(Warnings)

option(STRADA_BUILD_VIS "Build the Strada 2D Vis library and application" ${PROJECT_IS_TOP_LEVEL})
option(STRADA_BUILD_TESTS "Build tests for the Strada project" ${PROJECT_IS_TOP_LEVEL})
option(STRADA_ENABLE_LTO "Enable Link-Time Optimization (LTO/IPO) for Release builds" ON)
option(STRADA_BUILD_DOCS "Build documentation using Doxygen" ${PROJECT_IS_TOP_LEVEL})

include(LinkTimeOptimization)

if(STRADA_BUILD_DOCS)
  include(Doxygen)
  strada_enable_doxygen()
endif()

add_subdirectory(external)
add_subdirectory(src)
add_subdirectory(examples)

if(STRADA_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()

# Generate and install package config files
install(
  EXPORT StradaTargets
  FILE StradaTargets.cmake
  NAMESPACE Strada::
  DESTINATION ${STRADA_INSTALL_CMAKEDIR})

include(CMakePackageConfigHelpers)
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/StradaConfigVersion.cmake"
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY AnyNewerVersion)

configure_package_config_file(
  "${PROJECT_SOURCE_DIR}/cmake/Config.cmake.in" "${CMAKE_CURRENT_BINARY_DIR}/StradaConfig.cmake"
  INSTALL_DESTINATION ${STRADA_INSTALL_CMAKEDIR})

install(FILES "${CMAKE_CURRENT_BINARY_DIR}/StradaConfig.cmake" "${CMAKE_CURRENT_BINARY_DIR}/StradaConfigVersion.cmake"
        DESTINATION ${STRADA_INSTALL_CMAKEDIR})
