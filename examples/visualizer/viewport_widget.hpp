// SPDX-License-Identifier: BSL-1.0

#pragma once

#include <QOpenGLBuffer>
#include <QOpenGLExtraFunctions>
#include <QOpenGLShaderProgram>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLWidget>
#include <strada/vis/geometry_batcher.hpp>

namespace strada::vis {

/// Viewport widget subclassing QOpenGLWidget to render batched road geometry.
class ViewportWidget : public QOpenGLWidget, protected QOpenGLExtraFunctions {
  Q_OBJECT

 public:
  /// Constructor.
  explicit ViewportWidget(QWidget* parent = nullptr);

  /// Destructor.
  ~ViewportWidget() override;

  /// Sets the batched geometry to render.
  void SetGeometry(const BatchedGeometry& geometry);

 protected:
  /// Initializes OpenGL state, shaders, and buffers.
  void initializeGL() override;

  /// Resizes the viewport projection.
  void resizeGL(int w, int h) override;

  /// Renders the batched triangles and lines.
  void paintGL() override;

 private:
  BatchedGeometry geometry_;
  bool geometry_dirty_{false};

  QOpenGLShaderProgram shader_program_;

  QOpenGLVertexArrayObject triangles_vao_;
  QOpenGLBuffer triangles_vbo_{QOpenGLBuffer::VertexBuffer};
  QOpenGLBuffer triangles_ibo_{QOpenGLBuffer::IndexBuffer};

  QOpenGLVertexArrayObject lines_vao_;
  QOpenGLBuffer lines_vbo_{QOpenGLBuffer::VertexBuffer};

  // Viewport/Camera parameters (will be animated in Issue 28)
  float camera_x_{0.0f};
  float camera_y_{0.0f};
  float zoom_{1.0f};
  float rotation_{0.0f};

  void SetupTriangles();
  void SetupLines();
};

}  // namespace strada::vis
