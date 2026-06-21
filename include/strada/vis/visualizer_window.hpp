// SPDX-License-Identifier: BSL-1.0

#pragma once

#include <QMainWindow>
#include <string>

namespace strada::vis {

class ViewportWidget;

/// The main application window for the 2D visualizer.
class VisualizerWindow : public QMainWindow {
  Q_OBJECT

 public:
  /// Constructor.
  explicit VisualizerWindow(QWidget* parent = nullptr);

  /// Destructor.
  ~VisualizerWindow() override = default;

  /// Loads, parses, tessellates, and batches the given map path.
  void LoadMap(const std::string& file_path);

 private:
  ViewportWidget* viewport_{nullptr};
};

}  // namespace strada::vis
