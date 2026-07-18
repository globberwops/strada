#include <QFileInfo>
#include <QStatusBar>
#include <strada/strada.hpp>
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

    // Load and compile all layers through the Strada facade
    auto strada = Strada{std::filesystem::path{file_path}, {.chord_error = 0.1}};

    // Batch geometry from the Tessellator
    const auto tess_opt = strada.Tessellator();
    auto batched = BatchMapGeometry(tess_opt->get());

    // Update Viewport with layers from the facade
    viewport_->SetGeometry(batched, strada.AbstractSyntaxTree(), strada.CompiledPhysicsModel());

    // Update title and status bar
    const auto file_info = QFileInfo{QString::fromStdString(file_path)};
    setWindowTitle(QString("Strada 2D Map Visualizer - %1").arg(file_info.fileName()));
    statusBar()->showMessage(QString("Loaded %1 (%2 road meshes, %3 polylines)")
                                 .arg(file_info.fileName())
                                 .arg(tess_opt->get().Meshes().size())
                                 .arg(tess_opt->get().Polylines().size()));
  } catch (const std::exception& ex) {
    statusBar()->showMessage(QString("Error loading map: %1").arg(ex.what()));
  }
}

}  // namespace strada::vis
