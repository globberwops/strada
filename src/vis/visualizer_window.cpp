// SPDX-License-Identifier: BSL-1.0

#include <QFileInfo>
#include <QStatusBar>
#include <strada/cpm/compiled_physics_model.hpp>
#include <strada/parser/parser.hpp>
#include <strada/tess/tessellator.hpp>
#include <strada/vis/viewport_widget.hpp>
#include <strada/vis/visualizer_window.hpp>

namespace strada::vis {

VisualizerWindow::VisualizerWindow(QWidget* parent) : QMainWindow(parent) {
  // Set window properties
  setWindowTitle("Strada 2D Map Visualizer");
  resize(1024, 768);

  // Apply premium dark stylesheet
  setStyleSheet(R"(
    QMainWindow {
        background-color: #121418;
    }
    QStatusBar {
        background-color: #1a1d24;
        color: #a0aab8;
        font-family: 'Segoe UI', Arial, sans-serif;
        font-size: 12px;
        border-top: 1px solid #2d3340;
    }
  )");

  // Setup viewport
  viewport_ = new ViewportWidget(this);  // NOLINT
  setCentralWidget(viewport_);

  // Setup status bar
  statusBar()->showMessage("No map loaded. Drag and drop or specify map path.");
}

void VisualizerWindow::LoadMap(const std::string& file_path) {
  try {
    statusBar()->showMessage(QString("Loading map: %1...").arg(QString::fromStdString(file_path)));

    // 1. Parse XODR file
    auto map = parser::ParseFile(file_path);

    // 2. Build Tessellator
    const tess::Tessellator kTess(map, 0.1);  // Use 0.1m chord error for rendering quality

    // 3. Batch Geometry
    auto batched = BatchMapGeometry(kTess);

    // 4. Update Viewport
    viewport_->SetGeometry(batched, map, cpm::CompiledPhysicsModel::Build(map));

    // Update title and status bar
    const QFileInfo kFileInfo(QString::fromStdString(file_path));
    setWindowTitle(QString("Strada 2D Map Visualizer - %1").arg(kFileInfo.fileName()));
    statusBar()->showMessage(QString("Loaded %1 (%2 road meshes, %3 polylines)")
                                 .arg(kFileInfo.fileName())
                                 .arg(kTess.Meshes().size())
                                 .arg(kTess.Polylines().size()));
  } catch (const std::exception& ex) {
    statusBar()->showMessage(QString("Error loading map: %1").arg(ex.what()));
  }
}

}  // namespace strada::vis
