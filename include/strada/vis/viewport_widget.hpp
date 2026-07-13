#pragma once

#include <QObject>
#include <QOpenGLBuffer>
#include <QOpenGLExtraFunctions>
#include <QOpenGLShaderProgram>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLWidget>
#include <QPoint>
#include <optional>
#include <strada/ast/abstract_syntax_tree.hpp>
#include <strada/cpm/compiled_physics_model.hpp>
#include <strada/routing/graph.hpp>
#include <strada/vis/camera.hpp>
#include <strada/vis/geometry_batcher.hpp>
#include <string>

namespace strada::vis {

class ViewportWidget : public QOpenGLWidget, protected QOpenGLExtraFunctions {
  Q_OBJECT

 public:
  explicit ViewportWidget(QWidget* parent = nullptr);
  ~ViewportWidget() override;

  void SetGeometry(const BatchedGeometry& geometry, const ast::AbstractSyntaxTree& map,
                   cpm::CompiledPhysicsModel model);
  static auto FindActiveRoadType(const ast::Road& road, double s) -> ast::RoadType;

  auto IsRouteCreationMode() const -> bool;
  auto Waypoints() const -> const std::vector<std::string>&;
  auto ActiveRoute() const -> const std::optional<routing::Route>&;
  auto GetCamera() const -> const Camera&;

 protected:
  void initializeGL() override;
  void resizeGL(int w, int h) override;
  void paintGL() override;

  void mousePressEvent(QMouseEvent* event) override;
  void mouseMoveEvent(QMouseEvent* event) override;
  void mouseReleaseEvent(QMouseEvent* event) override;
  void wheelEvent(QWheelEvent* event) override;
  void keyPressEvent(QKeyEvent* event) override;
  auto event(QEvent* event) -> bool override;

 private:
  BatchedGeometry geometry_;
  bool geometry_dirty_{false};
  bool show_junction_boundaries_{false};
  bool show_border_lanes_{false};
  bool show_reference_lines_{true};
  bool show_objects_{false};
  bool show_signals_{false};
  bool show_lanes_{false};

  QOpenGLShaderProgram shader_program_;

  QOpenGLVertexArrayObject triangles_vao_;
  QOpenGLBuffer triangles_vbo_{QOpenGLBuffer::VertexBuffer};
  QOpenGLBuffer triangles_ibo_{QOpenGLBuffer::IndexBuffer};

  QOpenGLVertexArrayObject lines_vao_;
  QOpenGLBuffer lines_vbo_{QOpenGLBuffer::VertexBuffer};

  QOpenGLVertexArrayObject grid_vao_;
  QOpenGLBuffer grid_vbo_{QOpenGLBuffer::VertexBuffer};

  QOpenGLVertexArrayObject boundaries_vao_;
  QOpenGLBuffer boundaries_vbo_{QOpenGLBuffer::VertexBuffer};
  QOpenGLBuffer boundaries_ibo_{QOpenGLBuffer::IndexBuffer};

  QOpenGLVertexArrayObject objects_vao_;
  QOpenGLBuffer objects_vbo_{QOpenGLBuffer::VertexBuffer};

  QOpenGLVertexArrayObject signals_vao_;
  QOpenGLBuffer signals_vbo_{QOpenGLBuffer::VertexBuffer};

  Camera camera_;
  QPoint last_mouse_pos_;

  // Compiled Physics Model for CPU-side picking
  ast::AbstractSyntaxTree map_;
  cpm::CompiledPhysicsModel cpm_model_;
  bool has_model_{false};
  mutable cpm::QueryContext query_ctx_;
  QPoint mouse_press_pos_;

  // Hover tracking details
  std::optional<cpm::LanePose> hovered_pose_;
  std::string hovered_road_name_;
  int hovered_lane_original_id_{0};
  double hovered_x_{0.0};
  double hovered_y_{0.0};

  // Route Planning State
  bool route_creation_mode_{false};
  std::vector<std::string> waypoint_road_ids_;
  std::optional<routing::Route> active_route_;
  std::optional<routing::Graph> routing_graph_;

  auto IsDrivableLane(cpm::RoadId road_id, cpm::LaneId lane_id) const -> bool;
  void RecomputeRoute();

  void SetupTriangles();
  void SetupLines();
  void SetupBoundaries();
  void SetupObjects();
  void SetupSignals();

  void RenderGrid();
  void UpdateGeometryBuffers();
  void DrawScene();
  void DrawLaneInspector(QPainter& painter);
  void DrawCompass(QPainter& painter);
  void DrawScaleBar(QPainter& painter);
  void DrawShortcutsPanel(QPainter& painter);
  void DrawOverlays();
};

}  // namespace strada::vis
