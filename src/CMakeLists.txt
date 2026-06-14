add_library(strada parser/parser.cpp)

target_include_directories(
  strada PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
                 $<INSTALL_INTERFACE:include>)

target_link_libraries(strada PRIVATE pugixml)
