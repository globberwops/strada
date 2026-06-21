add_executable(test_strada ast/test_parser.cpp)

target_compile_definitions(
  test_strada PRIVATE STRADA_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/data")

target_link_libraries(test_strada PRIVATE Strada::strada GTest::gtest_main)

include(GoogleTest)
gtest_discover_tests(test_strada)

add_executable(
  test_cpm
  cpm/test_compiled_physics_model.cpp cpm/test_geometry_math.cpp
  cpm/test_reference_line.cpp cpm/test_rotation.cpp
  cpm/test_bounding_volume_hierarchy.cpp
  cpm/test_elevation_profile.cpp cpm/test_lane_network.cpp)
target_compile_definitions(
  test_cpm PRIVATE STRADA_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/data")
target_link_libraries(test_cpm PRIVATE Strada::strada GTest::gtest_main)
gtest_discover_tests(test_cpm)

add_executable(
  test_tess
  tess/test_tessellator.cpp)
target_compile_definitions(
  test_tess PRIVATE STRADA_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/data")
target_link_libraries(test_tess PRIVATE Strada::strada GTest::gtest_main)
gtest_discover_tests(test_tess)

if(STRADA_BUILD_VIS)
  add_subdirectory(vis)
endif()


