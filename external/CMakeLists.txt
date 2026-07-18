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

if(STRADA_BUILD_BENCHMARKS)
  set(BENCHMARK_ENABLE_TESTING
      OFF
      CACHE BOOL "" FORCE)
  set(BENCHMARK_ENABLE_INSTALL
      OFF
      CACHE BOOL "" FORCE)
  set(BENCHMARK_ENABLE_GTEST_TESTS
      OFF
      CACHE BOOL "" FORCE)

  FetchContent_Declare(
    googlebenchmark
    GIT_REPOSITORY https://github.com/google/benchmark.git
    GIT_TAG v1.9.0
    FIND_PACKAGE_ARGS NAMES benchmark GLOBAL)

  FetchContent_MakeAvailable(googlebenchmark)

  string(CONCAT asam_examples_url "https://publications.pages.asam.net/standards/ASAM_OpenDRIVE/"
                "ASAM_OpenDRIVE_Specification/latest/specification/_attachments/"
                "generated/ASAM_OpenDRIVE_v1-9-0_examples_and_use-cases.zip")

  FetchContent_Declare(
    asam_examples
    URL "${asam_examples_url}"
    URL_HASH SHA256=1142865e58efe4b700b11ba420f8ae1f751446b03f25f5071e7fcc1a8697d4c9)
  FetchContent_MakeAvailable(asam_examples)
endif()
