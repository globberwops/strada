# Dependencies via FetchContent
include(FetchContent)

# 1. pugixml
FetchContent_Declare(
  pugixml
  GIT_REPOSITORY https://github.com/zeux/pugixml.git
  GIT_TAG v1.14
  FIND_PACKAGE_ARGS GLOBAL)

# 1. Eigen
FetchContent_Declare(
  eigen
  GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
  GIT_TAG 3.4.0
  FIND_PACKAGE_ARGS NAMES Eigen3 GLOBAL)

# 1. GoogleTest
FetchContent_Declare(
  googletest
  GIT_REPOSITORY https://github.com/google/googletest.git
  GIT_TAG v1.17.0
  FIND_PACKAGE_ARGS NAMES GTest GLOBAL)

set(EIGEN_BUILD_TESTING
    OFF
    CACHE BOOL "" FORCE)
set(gtest_force_shared_crt
    ON
    CACHE BOOL "" FORCE)

FetchContent_MakeAvailable(pugixml eigen googletest)
