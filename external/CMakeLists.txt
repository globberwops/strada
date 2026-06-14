# Dependencies via FetchContent
include(FetchContent)

# 1. pugixml
FetchContent_Declare(
  pugixml
  URL https://github.com/zeux/pugixml/releases/download/v1.14/pugixml-1.14.tar.gz
  FIND_PACKAGE_ARGS GLOBAL)

# 2. Eigen
FetchContent_Declare(
  eigen URL https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz
  FIND_PACKAGE_ARGS NAMES Eigen3 GLOBAL)

# 3. GoogleTest
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.tar.gz
  FIND_PACKAGE_ARGS NAMES GTest GLOBAL)

set(EIGEN_BUILD_TESTING
    OFF
    CACHE BOOL "" FORCE)
set(gtest_force_shared_crt
    ON
    CACHE BOOL "" FORCE)

FetchContent_MakeAvailable(pugixml eigen googletest)
