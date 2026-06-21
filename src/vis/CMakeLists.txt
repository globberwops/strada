if(STRADA_BUILD_VIS)
  add_library(strada_vis)
  add_library(Strada::vis ALIAS strada_vis)

  set_target_properties(
    strada_vis
    PROPERTIES AUTOMOC ON
               AUTOUIC ON
               AUTORCC ON)

  target_sources(
    strada_vis
    PRIVATE geometry_batcher.cpp camera.cpp viewport_widget.cpp visualizer_window.cpp
    PUBLIC FILE_SET
           HEADERS
           BASE_DIRS
           ${PROJECT_SOURCE_DIR}/include
           FILES
           ${PROJECT_SOURCE_DIR}/include/strada/vis/geometry_batcher.hpp
           ${PROJECT_SOURCE_DIR}/include/strada/vis/camera.hpp
           ${PROJECT_SOURCE_DIR}/include/strada/vis/viewport_widget.hpp
           ${PROJECT_SOURCE_DIR}/include/strada/vis/visualizer_window.hpp)

  find_package(Qt6 REQUIRED COMPONENTS Core Gui Widgets OpenGLWidgets)
  find_package(OpenGL REQUIRED)

  target_link_libraries(strada_vis PUBLIC Strada::strada Qt6::Widgets Qt6::OpenGLWidgets OpenGL::GL)
endif()
