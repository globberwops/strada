include(FetchContent)

FetchContent_Declare(
  pugixml
  GIT_REPOSITORY https://github.com/zeux/pugixml.git
  GIT_TAG v1.14
  FIND_PACKAGE_ARGS GLOBAL)

FetchContent_MakeAvailable(pugixml)

if(STRADA_BUILD_TESTS)
  FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG v1.17.0
    FIND_PACKAGE_ARGS NAMES GTest GLOBAL)

  set(gtest_force_shared_crt
      ON
      CACHE BOOL "" FORCE)

  FetchContent_MakeAvailable(googletest)
endif()
