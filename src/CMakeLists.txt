add_library(strada parser/parser.cpp)
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
         ${PROJECT_SOURCE_DIR}/include/strada/ast/opendrive.hpp
         ${PROJECT_SOURCE_DIR}/include/strada/ast/profiles.hpp
         ${PROJECT_SOURCE_DIR}/include/strada/ast/road.hpp)

target_link_libraries(strada PRIVATE pugixml)
