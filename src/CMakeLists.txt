add_library(strada parser/parser.cpp cpm/compiled_physics_model.cpp)
add_library(Strada::strada ALIAS strada)

target_sources(
  strada
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
         ${PROJECT_SOURCE_DIR}/include/strada/cpm/ids.hpp
         ${PROJECT_SOURCE_DIR}/include/strada/cpm/coordinate.hpp
         ${PROJECT_SOURCE_DIR}/include/strada/cpm/query_context.hpp
         ${PROJECT_SOURCE_DIR}/include/strada/cpm/compiled_physics_model.hpp)


target_link_libraries(strada PRIVATE pugixml)
