# vis target sources for core strada library
target_sources(
  strada
  PRIVATE
    geometry_batcher.cpp
    camera.cpp
)

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
      viewport_widget.cpp
      visualizer_window.cpp
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
