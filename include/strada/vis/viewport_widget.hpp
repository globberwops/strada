// SPDX-License-Identifier: BSL-1.0

#pragma once

#include <QOpenGLBuffer>
#include <QOpenGLExtraFunctions>
#include <QOpenGLShaderProgram>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLWidget>
#include <QPoint>
#include <optional>
#include <strada/cpm/compiled_physics_model.hpp>
#include <strada/vis/camera.hpp>
#include <strada/vis/geometry_batcher.hpp>

namespace strada::vis {

class ViewportWidget : public QOpenGLWidget, protected QOpenGLExtraFunctions {
  Q_OBJECT

 public:
  explicit ViewportWidget(QWidget* parent = nullptr);
  ~ViewportWidget() override;

  void SetGeometry(const BatchedGeometry& geometry, cpm::CompiledPhysicsModel model);

 protected:
  void initializeGL() override;
  void resizeGL(int w, int h) override;
  void paintGL() override;

  void mousePressEvent(QMouseEvent* event) override;
  void mouseMoveEvent(QMouseEvent* event) override;
  void wheelEvent(QWheelEvent* event) override;
  void keyPressEvent(QKeyEvent* event) override;

 private:
  BatchedGeometry geometry_;
  bool geometry_dirty_{false};
  bool show_junction_boundaries_{true};

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

  Camera camera_;
  QPoint last_mouse_pos_;

  // Compiled Physics Model for CPU-side picking
  cpm::CompiledPhysicsModel cpm_model_;
  bool has_model_{false};
  mutable cpm::QueryContext query_ctx_;

  // Hover tracking details
  std::optional<cpm::LanePose> hovered_pose_;
  std::string hovered_road_name_;
  int hovered_lane_original_id_{0};

  void SetupTriangles();
  void SetupLines();
  void SetupBoundaries();

  void RenderGrid();
};

}  // namespace strada::vis
