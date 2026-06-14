add_executable(test_strada ast/test_parser.cpp)

target_compile_definitions(
  test_strada PRIVATE STRADA_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/data")

target_link_libraries(test_strada PRIVATE Strada::strada GTest::gtest_main)

# Enable CTest integration
include(GoogleTest)
gtest_discover_tests(test_strada)
