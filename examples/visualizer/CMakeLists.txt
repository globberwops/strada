find_package(Qt6 REQUIRED COMPONENTS Core Gui Widgets OpenGLWidgets)
find_package(OpenGL REQUIRED)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

add_executable(
  visualizer
  main.cpp
  visualizer_window.cpp
  viewport_widget.cpp)

target_link_libraries(
  visualizer
  PRIVATE Strada::strada
          Qt6::Widgets
          Qt6::OpenGLWidgets
          OpenGL::GL)
