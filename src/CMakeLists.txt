add_library(strada)
add_library(Strada::strada ALIAS strada)

target_sources(
  strada
  PRIVATE parser/parser.cpp
          cpm/compiled_physics_model.cpp
          cpm/geometry_math.cpp
          cpm/reference_line.cpp
          cpm/bounding_volume_hierarchy.cpp
          cpm/elevation_profile.cpp
          cpm/lane_network.cpp
          tess/tessellator.cpp
          vis/geometry_batcher.cpp
          vis/camera.cpp
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
         ${PROJECT_SOURCE_DIR}/include/strada/cpm/coordinate.hpp
         ${PROJECT_SOURCE_DIR}/include/strada/cpm/geometry_math.hpp
         ${PROJECT_SOURCE_DIR}/include/strada/cpm/bounding_volume_hierarchy.hpp
         ${PROJECT_SOURCE_DIR}/include/strada/cpm/ids.hpp
         ${PROJECT_SOURCE_DIR}/include/strada/cpm/query_context.hpp
         ${PROJECT_SOURCE_DIR}/include/strada/cpm/compiled_physics_model.hpp
         ${PROJECT_SOURCE_DIR}/include/strada/cpm/reference_line.hpp
         ${PROJECT_SOURCE_DIR}/include/strada/cpm/elevation_profile.hpp
         ${PROJECT_SOURCE_DIR}/include/strada/cpm/lane_network.hpp
         ${PROJECT_SOURCE_DIR}/include/strada/tess/tessellator.hpp
         ${PROJECT_SOURCE_DIR}/include/strada/vis/geometry_batcher.hpp
         ${PROJECT_SOURCE_DIR}/include/strada/vis/camera.hpp)

find_package(Qt6 REQUIRED COMPONENTS Core Gui)

target_link_libraries(strada PRIVATE pugixml PUBLIC Qt6::Core Qt6::Gui)

if(STRADA_BUILD_VISUALIZER)
  add_library(strada_visualizer)
  add_library(Strada::visualizer ALIAS strada_visualizer)

  set_target_properties(strada_visualizer PROPERTIES
    AUTOMOC ON
    AUTOUIC ON
    AUTORCC ON
  )

  target_sources(
    strada_visualizer
    PRIVATE
      vis/viewport_widget.cpp
      vis/visualizer_window.cpp
    PUBLIC FILE_SET
      HEADERS
      BASE_DIRS
        ${PROJECT_SOURCE_DIR}/include
      FILES
        ${PROJECT_SOURCE_DIR}/include/strada/vis/viewport_widget.hpp
        ${PROJECT_SOURCE_DIR}/include/strada/vis/visualizer_window.hpp
  )

  find_package(Qt6 REQUIRED COMPONENTS Core Gui Widgets OpenGLWidgets)
  find_package(OpenGL REQUIRED)

  target_link_libraries(
    strada_visualizer
    PUBLIC
      Strada::strada
      Qt6::Widgets
      Qt6::OpenGLWidgets
      OpenGL::GL
  )
endif()

