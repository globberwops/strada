// SPDX-License-Identifier: BSL-1.0

#include <QGestureEvent>
#include <QInputDevice>
#include <QKeyEvent>
#include <QMatrix4x4>
#include <QMouseEvent>
#include <QNativeGestureEvent>
#include <QPainter>
#include <QPinchGesture>
#include <QVector4D>
#include <QWheelEvent>
#include <algorithm>
#include <cmath>
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
  boundaries_vao_.destroy();
  boundaries_vbo_.destroy();
  boundaries_ibo_.destroy();
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
    camera_.camera_x = 0.5F * (min_x + max_x);
    camera_.camera_y = 0.5F * (min_y + max_y);

    float const dx = max_x - min_x;
    float const dy = max_y - min_y;
    float const max_dim = std::max(dx, dy);
    if (max_dim > 0.0F) {
      camera_.zoom = 300.0F / max_dim;
    } else {
      camera_.zoom = 10.0F;
    }
  }

  update();
}

void ViewportWidget::initializeGL() {
  initializeOpenGLFunctions();

  // Clear color matching Sleek Premium Dark Mode aesthetics
  glClearColor(0.1F, 0.12F, 0.15F, 1.0F);
  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_LEQUAL);

  // Set up Shader Program
  const char* vshader_source =
      R"(
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

  const char* fshader_source =
      R"(
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

  boundaries_vao_.create();
  boundaries_vbo_.create();
  boundaries_ibo_.create();
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
    SetupBoundaries();
    geometry_dirty_ = false;
  }

  shader_program_.bind();
  shader_program_.setUniformValue("useOverrideColor", 0);
  shader_program_.setUniformValue("projection", camera_.GetProjectionMatrix());
  shader_program_.setUniformValue("view", camera_.GetViewMatrix());

  // Draw Grid first in the background (no depth test/write)
  glDisable(GL_DEPTH_TEST);
  glDepthMask(GL_FALSE);
  RenderGrid();
  glDepthMask(GL_TRUE);
  glEnable(GL_DEPTH_TEST);

  // Draw Filled Junction Boundaries in the background (below lane meshes)
  if (show_junction_boundaries_ && !geometry_.boundary_triangle_indices.empty()) {
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    shader_program_.setUniformValue("useOverrideColor", 1);
    shader_program_.setUniformValue("overrideColor",
                                    QVector4D(245.0F / 255.0F, 197.0F / 255.0F, 61.0F / 255.0F, 0.12F));
    boundaries_vao_.bind();
    glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(geometry_.boundary_triangle_indices.size()), GL_UNSIGNED_INT,
                   nullptr);
    boundaries_vao_.release();
    shader_program_.setUniformValue("useOverrideColor", 0);
    glDisable(GL_BLEND);
  }

  // 1. Draw Road Surface Meshes in a Single batched call
  if (!geometry_.triangle_indices.empty()) {
    triangles_vao_.bind();
    glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(geometry_.triangle_indices.size()), GL_UNSIGNED_INT, nullptr);
    triangles_vao_.release();
  }

  // 2. Draw Boundaries/Markings in a Single batched call
  if (!geometry_.line_vertices.empty()) {
    glDisable(GL_DEPTH_TEST);
    lines_vao_.bind();
    glLineWidth(2.0F);
    glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(geometry_.line_vertices.size()));
    lines_vao_.release();
    glEnable(GL_DEPTH_TEST);
  }

  // 3. Draw Hover Highlight Overlay
  if (has_model_ && hovered_pose_) {
    for (const auto& range : geometry_.mesh_ranges) {
      if (range.road_id == hovered_pose_->road && range.lane_id == hovered_pose_->lane) {
        if (range.index_count > 0) {
          glDisable(GL_DEPTH_TEST);
          glEnable(GL_BLEND);
          glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

          shader_program_.setUniformValue("useOverrideColor", 1);
          shader_program_.setUniformValue("overrideColor", QVector4D(1.0F, 0.75F, 0.0F, 0.4F));

          triangles_vao_.bind();
          const void* offset =
              reinterpret_cast<const void*>(static_cast<uintptr_t>(range.index_start) * sizeof(std::uint32_t));
          glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(range.index_count), GL_UNSIGNED_INT, offset);
          triangles_vao_.release();

          shader_program_.setUniformValue("useOverrideColor", 0);
          glDisable(GL_BLEND);
          glEnable(GL_DEPTH_TEST);
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
      QRect const rect(20, 20, 260, 110);
      painter.setPen(QPen(QColor(45, 51, 64, 255), 1));
      painter.setBrush(QBrush(QColor(26, 29, 36, 220)));
      painter.drawRoundedRect(rect, 8.0, 8.0);

      // Setup font
      QFont font("Segoe UI", 10);
      painter.setFont(font);

      // Draw details
      int const x_offset = 35;
      int y_offset = 45;
      int const line_height = 22;

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
      QString const coords = QString("%1 m, %2 m").arg(hovered_pose_->s, 0, 'f', 3).arg(hovered_pose_->t, 0, 'f', 3);
      painter.drawText(x_offset + 85, y_offset, coords);
    }

    // 5. Draw Compass Gizmo in the top-right corner
    {
      int const cx = width() - 50;
      int const cy = 50;

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
      QFont const font("Segoe UI", 9, QFont::Bold);
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
      double const scale_length = CalculateScaleLength(camera_.zoom);
      double const s = scale_length * camera_.zoom;  // Width on screen

      int const num_segments = 4;
      double const seg_w = s / num_segments;
      double const x0 = width() - 20.0 - s;
      for (int i = 0; i < num_segments; ++i) {
        QRectF const seg_rect(x0 + (i * seg_w), height() - 35, seg_w, 8);
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
      QFont const font("Segoe UI", 9, QFont::Bold);
      painter.setFont(font);
      QString label;
      if (scale_length >= 1000.0) {
        label = QString("%1 km").arg(scale_length / 1000.0);
      } else {
        label = QString("%1 m").arg(scale_length);
      }
      painter.drawText(QRectF(x0, height() - 55, s, 15), Qt::AlignCenter, label);
    }

    // 7. Draw Keyboard Shortcuts Panel in the bottom-left corner
    {
      QRect const rect(20, height() - 170, 310, 150);
      painter.setPen(QPen(QColor(45, 51, 64, 255), 1));
      painter.setBrush(QBrush(QColor(26, 29, 36, 220)));
      painter.drawRoundedRect(rect, 8.0, 8.0);

      // Setup font
      QFont font("Segoe UI", 9);
      painter.setFont(font);

      int const x_offset = 35;
      int y_offset = height() - 145;
      int const line_height = 20;

      // Header
      font.setBold(true);
      painter.setFont(font);
      painter.setPen(QColor(245, 197, 61));  // Amber title color
      painter.drawText(x_offset, y_offset, "CONTROLS & SHORTCUTS");
      y_offset += 22;

      font.setBold(false);
      painter.setFont(font);

      struct ShortcutItem {
        QString key;
        QString desc;
      };
      std::vector<ShortcutItem> const items = {{"L-Click + Drag", "Pan Map"},
                                               {"R-Click + Drag", "Rotate Map"},
                                               {"Scroll Wheel", "Zoom Map"},
                                               {"R", "Reset View / Auto-fit"},
                                               {"J", "Toggle Junction Boundaries"}};

      for (const auto& item : items) {
        // Shortcut key
        painter.setPen(QColor(100, 181, 246));  // Light blue/cyan for keys
        painter.drawText(x_offset, y_offset, item.key);

        // Description
        painter.setPen(QColor(180, 188, 204));  // Slate white for description
        painter.drawText(x_offset + 95, y_offset, item.desc);

        y_offset += line_height;
      }
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

void ViewportWidget::SetupBoundaries() {
  boundaries_vao_.bind();

  // Load vertices
  boundaries_vbo_.bind();
  boundaries_vbo_.allocate(geometry_.boundary_triangle_vertices.data(),
                           static_cast<int>(geometry_.boundary_triangle_vertices.size() * sizeof(Vertex)));

  // Load indices
  boundaries_ibo_.bind();
  boundaries_ibo_.allocate(geometry_.boundary_triangle_indices.data(),
                           static_cast<int>(geometry_.boundary_triangle_indices.size() * sizeof(std::uint32_t)));

  // Set attribute locations
  shader_program_.enableAttributeArray(0);
  shader_program_.setAttributeBuffer(0, GL_FLOAT, offsetof(Vertex, x), 3, sizeof(Vertex));

  shader_program_.enableAttributeArray(1);
  shader_program_.setAttributeBuffer(1, GL_FLOAT, offsetof(Vertex, r), 3, sizeof(Vertex));

  boundaries_vao_.release();
}

void ViewportWidget::mousePressEvent(QMouseEvent* event) {
  last_mouse_pos_ = event->pos();
  setFocus();
}

void ViewportWidget::mouseMoveEvent(QMouseEvent* event) {
  QPoint const delta = event->pos() - last_mouse_pos_;
  last_mouse_pos_ = event->pos();

  if ((event->buttons() & Qt::LeftButton) != 0) {
    camera_.Pan(static_cast<float>(delta.x()), static_cast<float>(delta.y()));
  } else if ((event->buttons() & Qt::RightButton) != 0) {
    camera_.Rotate(static_cast<float>(delta.x()) * 0.5F);
  }

  // Hover picking detection (CPU-side via CPM)
  if (has_model_) {
    QPointF const world_pos =
        camera_.ScreenToWorld(static_cast<float>(event->position().x()), static_cast<float>(event->position().y()));

    cpm::InertialPose pose{};
    pose.x = world_pos.x();
    pose.y = world_pos.y();

    // Query closest vertex in rendering geometry to find road elevation at mouse cursor
    float best_z = 0.0F;
    if (!geometry_.triangle_vertices.empty()) {
      float min_dist_sq = std::numeric_limits<float>::max();
      for (const auto& v : geometry_.triangle_vertices) {
        float const dx = static_cast<float>(world_pos.x()) - v.x;
        float const dy = static_cast<float>(world_pos.y()) - v.y;
        float const dist_sq = (dx * dx) + (dy * dy);
        if (dist_sq < min_dist_sq) {
          min_dist_sq = dist_sq;
          best_z = v.z;
        }
      }
    }
    pose.z = best_z;

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
  // If the Ctrl modifier is NOT pressed and pixelDelta is populated,
  // we interpret this as a high-resolution touchpad two-finger scroll panning gesture.
  if (!(event->modifiers() & Qt::ControlModifier) && !event->pixelDelta().isNull()) {
    QPoint const delta = event->pixelDelta();
    camera_.Pan(static_cast<float>(delta.x()), static_cast<float>(delta.y()));
    update();
    return;
  }

  // Otherwise, it is a zoom event (mouse wheel or touchpad pinch Ctrl+scroll)
  QPoint const num_pixels = event->pixelDelta();
  QPoint const num_degrees = event->angleDelta() / 8;

  float factor = 1.0F;
  if (!num_pixels.isNull()) {
    factor = std::pow(1.15F, static_cast<float>(num_pixels.y()) * 0.05F);
  } else if (!num_degrees.isNull()) {
    factor = std::pow(1.15F, static_cast<float>(num_degrees.y()) / 15.0F);
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
      camera_.camera_x = 0.5F * (min_x + max_x);
      camera_.camera_y = 0.5F * (min_y + max_y);
      float const dx = max_x - min_x;
      float const dy = max_y - min_y;
      float const max_dim = std::max(dx, dy);
      if (max_dim > 0.0F) {
        camera_.zoom = 300.0F / max_dim;
      }
    }
    update();
  } else if (event->key() == Qt::Key_J) {
    show_junction_boundaries_ = !show_junction_boundaries_;
    update();
  }
}

auto ViewportWidget::event(QEvent* event) -> bool {
  if (event->type() == QEvent::NativeGesture) {
    auto* gesture_event = dynamic_cast<QNativeGestureEvent*>(event);
    if (gesture_event->gestureType() == Qt::ZoomNativeGesture) {
      qreal const val = gesture_event->value();
      QPointF const pos = gesture_event->position();
      camera_.ZoomAt(static_cast<float>(pos.x()), static_cast<float>(pos.y()), static_cast<float>(1.0 + val));
      update();
      return true;
    }
    if (gesture_event->gestureType() == Qt::RotateNativeGesture) {
      qreal const val = gesture_event->value();
      camera_.Rotate(-static_cast<float>(val));
      update();
      return true;
    }
  }
  return QOpenGLWidget::event(event);
}

void ViewportWidget::RenderGrid() {
  double const scale_length = CalculateScaleLength(camera_.zoom);
  if (scale_length <= 0.0) {
    return;
  }

  // Calculate maximum visible bounds in world coordinates
  auto w = static_cast<float>(width());
  auto h = static_cast<float>(height());
  float const r_screen = std::sqrt((w * w) + (h * h)) / 2.0F;
  float const r_world = r_screen / camera_.zoom;

  float const min_x = camera_.camera_x - r_world;
  float const max_x = camera_.camera_x + r_world;
  float const min_y = camera_.camera_y - r_world;
  float const max_y = camera_.camera_y + r_world;

  auto grid_spacing = static_cast<float>(scale_length);

  std::vector<Vertex> grid_vertices;

  // Grid line color: subtle dark grey-blue
  float const r = 0.16F;
  float const g = 0.18F;
  float const b = 0.22F;

  // Vertical lines (constant x)
  float const start_x = std::floor(min_x / grid_spacing) * grid_spacing;
  float const end_x = std::ceil(max_x / grid_spacing) * grid_spacing;
  for (float x = start_x; x <= end_x; x += grid_spacing) {
    grid_vertices.push_back(Vertex{.x = x, .y = min_y, .z = 0.0F, .r = r, .g = g, .b = b});
    grid_vertices.push_back(Vertex{.x = x, .y = max_y, .z = 0.0F, .r = r, .g = g, .b = b});
  }

  // Horizontal lines (constant y)
  float const start_y = std::floor(min_y / grid_spacing) * grid_spacing;
  float const end_y = std::ceil(max_y / grid_spacing) * grid_spacing;
  for (float y = start_y; y <= end_y; y += grid_spacing) {
    grid_vertices.push_back(Vertex{.x = min_x, .y = y, .z = 0.0F, .r = r, .g = g, .b = b});
    grid_vertices.push_back(Vertex{.x = max_x, .y = y, .z = 0.0F, .r = r, .g = g, .b = b});
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
  glLineWidth(1.0F);
  glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(grid_vertices.size()));

  grid_vao_.release();
}

}  // namespace strada::vis
