// SPDX-License-Identifier: BSL-1.0

#include <QApplication>
#include <filesystem>
#include <iostream>

#include <strada/vis/visualizer_window.hpp>

int main(int argc, char* argv[]) {
  QApplication app(argc, argv);

  strada::vis::VisualizerWindow window;
  window.show();

  // Load map if specified, otherwise search for a default test map
  std::string map_path;
  if (argc > 1) {
    map_path = argv[1];
  } else {
    // Look for default flat map in standard test location
    std::vector<std::string> search_paths = {"tests/data/lanes_and_profiles.xodr",
                                             "../tests/data/lanes_and_profiles.xodr",
                                             "../../tests/data/lanes_and_profiles.xodr"};
    for (const auto& path : search_paths) {
      if (std::filesystem::exists(path)) {
        map_path = path;
        break;
      }
    }
  }

  if (!map_path.empty()) {
    std::cout << "Loading map file: " << map_path << std::endl;
    window.LoadMap(map_path);
  } else {
    std::cout << "No default map found. Open window with map filename argument." << std::endl;
  }

  return app.exec();
}
