add_library(strada)
add_library(Strada::strada ALIAS strada)

target_sources(
  strada
  PRIVATE parser/parser.cpp
          cpm/compiled_physics_model.cpp
          cpm/road_projector.cpp
          cpm/geometry_math.cpp
          cpm/reference_line.cpp
          cpm/bounding_volume_hierarchy.cpp
          cpm/elevation_profile.cpp
          cpm/lane_network.cpp
          cpm/polynomials.cpp
          tess/tessellator.cpp
          tess/triangulation.cpp
  PUBLIC FILE_SET
         HEADERS
         BASE_DIRS
         ${PROJECT_SOURCE_DIR}/include
         FILES
         ${PROJECT_SOURCE_DIR}/include/strada/parser/parser.hpp
         ${PROJECT_SOURCE_DIR}/include/strada/ast/geometry.hpp
         ${PROJECT_SOURCE_DIR}/include/strada/ast/header.hpp
         ${PROJECT_SOURCE_DIR}/include/strada/ast/lanes.hpp
         ${PROJECT_SOURCE_DIR}/include/strada/ast/abstract_syntax_tree.hpp
         ${PROJECT_SOURCE_DIR}/include/strada/ast/profiles.hpp
         ${PROJECT_SOURCE_DIR}/include/strada/ast/road.hpp
         ${PROJECT_SOURCE_DIR}/include/strada/ast/signals.hpp
         ${PROJECT_SOURCE_DIR}/include/strada/cpm/coordinate.hpp
         ${PROJECT_SOURCE_DIR}/include/strada/cpm/geometry_math.hpp
         ${PROJECT_SOURCE_DIR}/include/strada/cpm/bounding_volume_hierarchy.hpp
         ${PROJECT_SOURCE_DIR}/include/strada/cpm/ids.hpp
         ${PROJECT_SOURCE_DIR}/include/strada/cpm/query_context.hpp
         ${PROJECT_SOURCE_DIR}/include/strada/cpm/compiled_physics_model.hpp
         ${PROJECT_SOURCE_DIR}/include/strada/cpm/reference_line.hpp
         ${PROJECT_SOURCE_DIR}/include/strada/cpm/elevation_profile.hpp
         ${PROJECT_SOURCE_DIR}/include/strada/cpm/lane_network.hpp
         ${PROJECT_SOURCE_DIR}/include/strada/cpm/polynomials.hpp
         ${PROJECT_SOURCE_DIR}/include/strada/tess/tessellator.hpp)

target_link_libraries(strada PRIVATE pugixml)

strada_add_warnings(strada)
strada_enable_lto(strada)

install(
  TARGETS strada
  EXPORT StradaTargets
  FILE_SET HEADERS
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

if(STRADA_BUILD_VIS)
  add_subdirectory(vis)
endif()
