// SPDX-License-Identifier: BSL-1.0

#pragma once

#include <QOpenGLBuffer>
#include <QOpenGLExtraFunctions>
#include <QOpenGLShaderProgram>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLWidget>
#include <strada/vis/geometry_batcher.hpp>
#include <QPoint>
#include <strada/vis/camera.hpp>

namespace strada::vis {

class ViewportWidget : public QOpenGLWidget, protected QOpenGLExtraFunctions {
  Q_OBJECT

 public:
  explicit ViewportWidget(QWidget* parent = nullptr);
  ~ViewportWidget() override;

  void SetGeometry(const BatchedGeometry& geometry);

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

  QOpenGLShaderProgram shader_program_;

  QOpenGLVertexArrayObject triangles_vao_;
  QOpenGLBuffer triangles_vbo_{QOpenGLBuffer::VertexBuffer};
  QOpenGLBuffer triangles_ibo_{QOpenGLBuffer::IndexBuffer};

  QOpenGLVertexArrayObject lines_vao_;
  QOpenGLBuffer lines_vbo_{QOpenGLBuffer::VertexBuffer};

  Camera camera_;
  QPoint last_mouse_pos_;

  void SetupTriangles();
  void SetupLines();
};

}  // namespace strada::vis
