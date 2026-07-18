cmake_minimum_required(VERSION 3.24)
project(StradaTidyPlugin LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Find LLVM and Clang packages installed on the system
find_package(LLVM 21.1 REQUIRED CONFIG PATHS "/usr/lib/llvm-21/lib/cmake/llvm")
find_package(Clang 21.1 REQUIRED CONFIG PATHS "/usr/lib/llvm-21/lib/cmake/clang")

message(STATUS "Found LLVM ${LLVM_PACKAGE_VERSION}")
message(STATUS "Using LLVMConfig.cmake in: ${LLVM_DIR}")

include_directories(${LLVM_INCLUDE_DIRS} ${CLANG_INCLUDE_DIRS})
link_directories(${LLVM_LIBRARY_DIRS})
add_definitions(${LLVM_DEFINITIONS})

# Add the shared library plugin target
add_library(strada_tidy_plugin MODULE StradaTidyModule.cpp LocalAutoStyleCheck.cpp)

# Avoid symbol export limitations and link against libclang-cpp
target_link_libraries(strada_tidy_plugin PRIVATE clang-cpp LLVMSupport)

# Position-independent code is required for shared libraries/modules
set_target_properties(
  strada_tidy_plugin
  PROPERTIES POSITION_INDEPENDENT_CODE ON
             PREFIX "lib"
             SUFFIX ".so")
