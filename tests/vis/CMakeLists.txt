add_executable(
  test_vis
  test_vis.cpp)
target_compile_definitions(
  test_vis PRIVATE STRADA_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../data")
target_link_libraries(test_vis PRIVATE Strada::vis GTest::gtest_main)
strada_add_warnings(test_vis)
gtest_discover_tests(test_vis)

add_executable(
  test_camera
  test_camera.cpp)
target_compile_definitions(
  test_camera PRIVATE STRADA_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../data")
target_link_libraries(test_camera PRIVATE Strada::vis GTest::gtest_main)
strada_add_warnings(test_camera)
gtest_discover_tests(test_camera)
