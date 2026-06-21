// SPDX-License-Identifier: BSL-1.0

#include <QApplication>
#include <iostream>
#include <strada/vis/visualizer_window.hpp>

auto main(int argc, char* argv[]) -> int {
  QApplication app(argc, argv);

  strada::vis::VisualizerWindow window;
  window.show();

  // Load map only if explicitly specified via command line arguments
  if (argc > 1) {
    std::string map_path = argv[1];
    std::cout << "Loading map file: " << map_path << '\n';
    window.LoadMap(map_path);
  } else {
    std::cout << "No map file specified. Open window with map filename argument to load." << '\n';
  }

  return QApplication::exec();
}
