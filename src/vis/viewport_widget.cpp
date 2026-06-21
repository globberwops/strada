// SPDX-License-Identifier: BSL-1.0

#include <QKeyEvent>
#include <QMatrix4x4>
#include <QMouseEvent>
#include <QPainter>
#include <QVector4D>
#include <QWheelEvent>
#include <algorithm>
#include <limits>
#include <strada/vis/viewport_widget.hpp>

namespace strada::vis {

ViewportWidget::ViewportWidget(QWidget* parent) : QOpenGLWidget(parent) {
  setFocusPolicy(Qt::StrongFocus);
  setMouseTracking(true);
}

ViewportWidget::~ViewportWidget() {
  makeCurrent();
  triangles_vao_.destroy();
  triangles_vbo_.destroy();
  triangles_ibo_.destroy();
  lines_vao_.destroy();
  lines_vbo_.destroy();
  grid_vao_.destroy();
  grid_vbo_.destroy();
  doneCurrent();
}

void ViewportWidget::SetGeometry(const BatchedGeometry& geometry, cpm::CompiledPhysicsModel model) {
  geometry_ = geometry;
  cpm_model_ = std::move(model);
  has_model_ = true;
  hovered_pose_ = std::nullopt;
  geometry_dirty_ = true;

  // Compute bounding box to center the camera automatically on load
  float min_x = std::numeric_limits<float>::max();
  float max_x = std::numeric_limits<float>::lowest();
  float min_y = std::numeric_limits<float>::max();
  float max_y = std::numeric_limits<float>::lowest();

  for (const auto& v : geometry_.triangle_vertices) {
    min_x = std::min(min_x, v.x);
    max_x = std::max(max_x, v.x);
    min_y = std::min(min_y, v.y);
    max_y = std::max(max_y, v.y);
  }

  for (const auto& v : geometry_.line_vertices) {
    min_x = std::min(min_x, v.x);
    max_x = std::max(max_x, v.x);
    min_y = std::min(min_y, v.y);
    max_y = std::max(max_y, v.y);
  }

  if (max_x >= min_x && max_y >= min_y) {
    camera_.camera_x = 0.5f * (min_x + max_x);
    camera_.camera_y = 0.5f * (min_y + max_y);

    float dx = max_x - min_x;
    float dy = max_y - min_y;
    float max_dim = std::max(dx, dy);
    if (max_dim > 0.0f) {
      camera_.zoom = 300.0f / max_dim;
    } else {
      camera_.zoom = 10.0f;
    }
  }

  update();
}

void ViewportWidget::initializeGL() {
  initializeOpenGLFunctions();

  // Clear color matching Sleek Premium Dark Mode aesthetics
  glClearColor(0.1f, 0.12f, 0.15f, 1.0f);
  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_LEQUAL);

  // Set up Shader Program
  const char* vshader_source = R"(
    #version 330 core
    layout(location = 0) in vec3 aPos;
    layout(location = 1) in vec3 aColor;
    out vec3 ourColor;
    uniform mat4 projection;
    uniform mat4 view;
    void main() {
        gl_Position = projection * view * vec4(aPos, 1.0);
        ourColor = aColor;
    }
  )";

  const char* fshader_source = R"(
    #version 330 core
    out vec4 FragColor;
    in vec3 ourColor;
    uniform vec4 overrideColor;
    uniform bool useOverrideColor;
    void main() {
        if (useOverrideColor) {
            FragColor = overrideColor;
        } else {
            FragColor = vec4(ourColor, 1.0);
        }
    }
  )";

  shader_program_.addShaderFromSourceCode(QOpenGLShader::Vertex, vshader_source);
  shader_program_.addShaderFromSourceCode(QOpenGLShader::Fragment, fshader_source);
  shader_program_.link();

  // Initialize VAOs and buffers
  triangles_vao_.create();
  triangles_vbo_.create();
  triangles_ibo_.create();

  lines_vao_.create();
  lines_vbo_.create();

  grid_vao_.create();
  grid_vbo_.create();
}

void ViewportWidget::resizeGL(int w, int h) {
  glViewport(0, 0, w, h);
  camera_.SetViewport(w, h);
}

void ViewportWidget::paintGL() {
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  if (geometry_dirty_) {
    SetupTriangles();
    SetupLines();
    geometry_dirty_ = false;
  }

  shader_program_.bind();
  shader_program_.setUniformValue("useOverrideColor", false);
  shader_program_.setUniformValue("projection", camera_.GetProjectionMatrix());
  shader_program_.setUniformValue("view", camera_.GetViewMatrix());

  // Draw Grid first in the background (no depth test/write)
  glDisable(GL_DEPTH_TEST);
  glDepthMask(GL_FALSE);
  RenderGrid();
  glDepthMask(GL_TRUE);
  glEnable(GL_DEPTH_TEST);

  // 1. Draw Road Surface Meshes in a Single batched call
  if (!geometry_.triangle_indices.empty()) {
    triangles_vao_.bind();
    glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(geometry_.triangle_indices.size()), GL_UNSIGNED_INT, nullptr);
    triangles_vao_.release();
  }

  // 2. Draw Boundaries/Markings in a Single batched call
  if (!geometry_.line_vertices.empty()) {
    lines_vao_.bind();
    glLineWidth(2.0f);
    glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(geometry_.line_vertices.size()));
    lines_vao_.release();
  }

  // 3. Draw Hover Highlight Overlay
  if (has_model_ && hovered_pose_) {
    for (const auto& range : geometry_.mesh_ranges) {
      if (range.road_id == hovered_pose_->road && range.lane_id == hovered_pose_->lane) {
        if (range.index_count > 0) {
          glEnable(GL_BLEND);
          glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

          shader_program_.setUniformValue("useOverrideColor", true);
          shader_program_.setUniformValue("overrideColor", QVector4D(1.0f, 0.75f, 0.0f, 0.4f));

          triangles_vao_.bind();
          const void* offset =
              reinterpret_cast<const void*>(static_cast<uintptr_t>(range.index_start) * sizeof(std::uint32_t));
          glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(range.index_count), GL_UNSIGNED_INT, offset);
          triangles_vao_.release();

          shader_program_.setUniformValue("useOverrideColor", false);
          glDisable(GL_BLEND);
        }
        break;
      }
    }
  }

  shader_program_.release();

  // 4. Draw QPainter overlays (HUD, Compass, Scale Bar)
  {
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);

    // Draw Lane Inspector HUD card if hovered
    if (has_model_ && hovered_pose_) {
      // Draw dark glassmorphic card container in the top-left corner
      QRect rect(20, 20, 260, 110);
      painter.setPen(QPen(QColor(45, 51, 64, 255), 1));
      painter.setBrush(QBrush(QColor(26, 29, 36, 220)));
      painter.drawRoundedRect(rect, 8.0, 8.0);

      // Setup font
      QFont font("Segoe UI", 10);
      painter.setFont(font);

      // Draw details
      int x_offset = 35;
      int y_offset = 45;
      int line_height = 22;

      // Header / Title
      font.setBold(true);
      painter.setFont(font);
      painter.setPen(QColor(255, 204, 0));  // Gold title color matching highlight
      painter.drawText(x_offset, y_offset, "LANE INSPECTOR");

      font.setBold(false);
      painter.setFont(font);
      y_offset += line_height;

      // Road ID
      painter.setPen(QColor(160, 170, 184));
      painter.drawText(x_offset, y_offset, "Road ID:");
      painter.setPen(Qt::white);
      painter.drawText(x_offset + 70, y_offset, QString::fromStdString(hovered_road_name_));
      y_offset += line_height;

      // Lane ID
      painter.setPen(QColor(160, 170, 184));
      painter.drawText(x_offset, y_offset, "Lane ID:");
      painter.setPen(Qt::white);
      painter.drawText(x_offset + 70, y_offset, QString::number(hovered_lane_original_id_));
      y_offset += line_height;

      // Coordinates (s, t)
      painter.setPen(QColor(160, 170, 184));
      painter.drawText(x_offset, y_offset, "Track (s, t):");
      painter.setPen(QColor(100, 181, 246));  // Light blue for values
      QString coords = QString("%1 m, %2 m").arg(hovered_pose_->s, 0, 'f', 3).arg(hovered_pose_->t, 0, 'f', 3);
      painter.drawText(x_offset + 85, y_offset, coords);
    }

    // 5. Draw Compass Gizmo in the top-right corner
    {
      int cx = width() - 50;
      int cy = 50;

      // Draw dark glassmorphic circular background
      painter.setPen(QPen(QColor(45, 51, 64, 255), 1));
      painter.setBrush(QBrush(QColor(26, 29, 36, 220)));
      painter.drawEllipse(QPoint(cx, cy), 28, 28);

      // Save painter state to apply rotation
      painter.save();
      painter.translate(cx, cy);
      painter.rotate(-camera_.rotation);

      // Draw East axis (positive X) - Slate Blue / Premium Cyan
      painter.setPen(QPen(QColor(100, 181, 246, 255), 2));
      painter.drawLine(0, 0, 20, 0);

      // Draw North axis (positive Y, which is up, so -20 in QPainter screen Y) - Red / Premium Coral
      painter.setPen(QPen(QColor(255, 110, 110, 255), 2));
      painter.drawLine(0, 0, 0, -20);

      // Draw Labels E and N
      QFont font("Segoe UI", 9, QFont::Bold);
      painter.setFont(font);

      // E Label
      painter.setPen(QColor(100, 181, 246));
      painter.drawText(QRect(22, -8, 16, 16), Qt::AlignCenter, "E");

      // N Label
      painter.setPen(QColor(255, 110, 110));
      painter.drawText(QRect(-8, -36, 16, 16), Qt::AlignCenter, "N");

      painter.restore();
    }

    // 6. Draw Geographical Scale Bar in the bottom-right corner
    {
      double scale_length = CalculateScaleLength(camera_.zoom);
      double S = scale_length * camera_.zoom;  // Width on screen

      int num_segments = 4;
      double seg_w = S / num_segments;
      double x0 = width() - 20.0 - S;
      for (int i = 0; i < num_segments; ++i) {
        QRectF seg_rect(x0 + i * seg_w, height() - 35, seg_w, 8);
        if (i % 2 == 0) {
          painter.setBrush(QBrush(QColor(26, 29, 36)));  // Filled dark
        } else {
          painter.setBrush(QBrush(QColor(240, 240, 240)));  // Light segment
        }
        painter.setPen(QPen(QColor(240, 240, 240), 1));  // White border for contrast
        painter.drawRect(seg_rect);
      }

      // Draw text label centered above the scale bar
      painter.setPen(QColor(240, 240, 240));
      QFont font("Segoe UI", 9, QFont::Bold);
      painter.setFont(font);
      QString label;
      if (scale_length >= 1000.0) {
        label = QString("%1 km").arg(scale_length / 1000.0);
      } else {
        label = QString("%1 m").arg(scale_length);
      }
      painter.drawText(QRectF(x0, height() - 55, S, 15), Qt::AlignCenter, label);
    }
  }
}

void ViewportWidget::SetupTriangles() {
  triangles_vao_.bind();

  // Load vertices
  triangles_vbo_.bind();
  triangles_vbo_.allocate(geometry_.triangle_vertices.data(),
                          static_cast<int>(geometry_.triangle_vertices.size() * sizeof(Vertex)));

  // Load indices
  triangles_ibo_.bind();
  triangles_ibo_.allocate(geometry_.triangle_indices.data(),
                          static_cast<int>(geometry_.triangle_indices.size() * sizeof(std::uint32_t)));

  // Set attribute locations
  shader_program_.enableAttributeArray(0);
  shader_program_.setAttributeBuffer(0, GL_FLOAT, offsetof(Vertex, x), 3, sizeof(Vertex));

  shader_program_.enableAttributeArray(1);
  shader_program_.setAttributeBuffer(1, GL_FLOAT, offsetof(Vertex, r), 3, sizeof(Vertex));

  triangles_vao_.release();
}

void ViewportWidget::SetupLines() {
  lines_vao_.bind();

  // Load line vertices
  lines_vbo_.bind();
  lines_vbo_.allocate(geometry_.line_vertices.data(),
                      static_cast<int>(geometry_.line_vertices.size() * sizeof(Vertex)));

  // Set attribute locations
  shader_program_.enableAttributeArray(0);
  shader_program_.setAttributeBuffer(0, GL_FLOAT, offsetof(Vertex, x), 3, sizeof(Vertex));

  shader_program_.enableAttributeArray(1);
  shader_program_.setAttributeBuffer(1, GL_FLOAT, offsetof(Vertex, r), 3, sizeof(Vertex));

  lines_vao_.release();
}

void ViewportWidget::mousePressEvent(QMouseEvent* event) {
  last_mouse_pos_ = event->pos();
  setFocus();
}

void ViewportWidget::mouseMoveEvent(QMouseEvent* event) {
  QPoint delta = event->pos() - last_mouse_pos_;
  last_mouse_pos_ = event->pos();

  if (event->buttons() & Qt::LeftButton) {
    camera_.Pan(static_cast<float>(delta.x()), static_cast<float>(delta.y()));
  } else if (event->buttons() & Qt::RightButton) {
    camera_.Rotate(static_cast<float>(delta.x()) * 0.5f);
  }

  // Hover picking detection (CPU-side via CPM)
  if (has_model_) {
    QPointF world_pos =
        camera_.ScreenToWorld(static_cast<float>(event->position().x()), static_cast<float>(event->position().y()));

    cpm::InertialPose pose{};
    pose.x = world_pos.x();
    pose.y = world_pos.y();
    pose.z = 0.0;
    pose.heading = 0.0;
    pose.pitch = 0.0;
    pose.roll = 0.0;

    auto lp_opt = cpm_model_.InertialToLane(pose, query_ctx_);
    if (lp_opt.has_value()) {
      hovered_pose_ = *lp_opt;
      hovered_road_name_ = std::string(cpm_model_.OriginalRoadId(hovered_pose_->road));
      hovered_lane_original_id_ = cpm_model_.OriginalLaneId(hovered_pose_->lane);
    } else {
      hovered_pose_ = std::nullopt;
    }
  }

  update();
}

void ViewportWidget::wheelEvent(QWheelEvent* event) {
  QPoint num_pixels = event->pixelDelta();
  QPoint num_degrees = event->angleDelta() / 8;

  float factor = 1.0f;
  if (!num_pixels.isNull()) {
    factor = 1.0f + (static_cast<float>(num_pixels.y()) * 0.005f);
  } else if (!num_degrees.isNull()) {
    factor = 1.0f + (static_cast<float>(num_degrees.y()) / 120.0f * 0.1f);
  }

  camera_.ZoomAt(static_cast<float>(event->position().x()), static_cast<float>(event->position().y()), factor);
  update();
}

void ViewportWidget::keyPressEvent(QKeyEvent* event) {
  if (event->key() == Qt::Key_R) {
    camera_.Reset();
    float min_x = std::numeric_limits<float>::max();
    float max_x = std::numeric_limits<float>::lowest();
    float min_y = std::numeric_limits<float>::max();
    float max_y = std::numeric_limits<float>::lowest();
    for (const auto& v : geometry_.triangle_vertices) {
      min_x = std::min(min_x, v.x);
      max_x = std::max(max_x, v.x);
      min_y = std::min(min_y, v.y);
      max_y = std::max(max_y, v.y);
    }
    for (const auto& v : geometry_.line_vertices) {
      min_x = std::min(min_x, v.x);
      max_x = std::max(max_x, v.x);
      min_y = std::min(min_y, v.y);
      max_y = std::max(max_y, v.y);
    }
    if (max_x >= min_x && max_y >= min_y) {
      camera_.camera_x = 0.5f * (min_x + max_x);
      camera_.camera_y = 0.5f * (min_y + max_y);
      float dx = max_x - min_x;
      float dy = max_y - min_y;
      float max_dim = std::max(dx, dy);
      if (max_dim > 0.0f) {
        camera_.zoom = 300.0f / max_dim;
      }
    }
    update();
  }
}

void ViewportWidget::RenderGrid() {
  double scale_length = CalculateScaleLength(camera_.zoom);
  if (scale_length <= 0.0) {
    return;
  }

  // Calculate maximum visible bounds in world coordinates
  float W = static_cast<float>(width());
  float H = static_cast<float>(height());
  float R_screen = std::sqrt(W * W + H * H) / 2.0f;
  float R_world = R_screen / camera_.zoom;

  float min_x = camera_.camera_x - R_world;
  float max_x = camera_.camera_x + R_world;
  float min_y = camera_.camera_y - R_world;
  float max_y = camera_.camera_y + R_world;

  float G = static_cast<float>(scale_length);

  std::vector<Vertex> grid_vertices;

  // Grid line color: subtle dark grey-blue
  float r = 0.16f;
  float g = 0.18f;
  float b = 0.22f;

  // Vertical lines (constant x)
  float start_x = std::floor(min_x / G) * G;
  float end_x = std::ceil(max_x / G) * G;
  for (float x = start_x; x <= end_x; x += G) {
    grid_vertices.push_back(Vertex{.x = x, .y = min_y, .z = 0.0f, .r = r, .g = g, .b = b});
    grid_vertices.push_back(Vertex{.x = x, .y = max_y, .z = 0.0f, .r = r, .g = g, .b = b});
  }

  // Horizontal lines (constant y)
  float start_y = std::floor(min_y / G) * G;
  float end_y = std::ceil(max_y / G) * G;
  for (float y = start_y; y <= end_y; y += G) {
    grid_vertices.push_back(Vertex{.x = min_x, .y = y, .z = 0.0f, .r = r, .g = g, .b = b});
    grid_vertices.push_back(Vertex{.x = max_x, .y = y, .z = 0.0f, .r = r, .g = g, .b = b});
  }

  if (grid_vertices.empty()) {
    return;
  }

  // Bind VAO and upload data
  grid_vao_.bind();
  grid_vbo_.bind();
  grid_vbo_.allocate(grid_vertices.data(), static_cast<int>(grid_vertices.size() * sizeof(Vertex)));

  // Setup attributes
  shader_program_.enableAttributeArray(0);
  shader_program_.setAttributeBuffer(0, GL_FLOAT, offsetof(Vertex, x), 3, sizeof(Vertex));
  shader_program_.enableAttributeArray(1);
  shader_program_.setAttributeBuffer(1, GL_FLOAT, offsetof(Vertex, r), 3, sizeof(Vertex));

  // Draw lines
  glLineWidth(1.0f);
  glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(grid_vertices.size()));

  grid_vao_.release();
}

}  // namespace strada::vis
