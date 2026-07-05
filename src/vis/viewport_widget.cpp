// SPDX-License-Identifier: BSL-1.0

#include <QGestureEvent>
#include <QInputDevice>
#include <QKeyEvent>
#include <QMainWindow>
#include <QMatrix4x4>
#include <QMouseEvent>
#include <QNativeGestureEvent>
#include <QPainter>
#include <QPinchGesture>
#include <QStatusBar>
#include <QVector4D>
#include <QWheelEvent>
#include <algorithm>
#include <cmath>
#include <limits>
#include <strada/parser/conversions.hpp>
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
  objects_vao_.destroy();
  objects_vbo_.destroy();
  doneCurrent();
}

void ViewportWidget::SetGeometry(const BatchedGeometry& geometry, const ast::AbstractSyntaxTree& map,
                                 cpm::CompiledPhysicsModel model) {
  geometry_ = geometry;
  map_ = map;
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

  for (const auto& v : geometry_.object_line_vertices) {
    min_x = std::min(min_x, v.x);
    max_x = std::max(max_x, v.x);
    min_y = std::min(min_y, v.y);
    max_y = std::max(max_y, v.y);
  }

  if (max_x >= min_x && max_y >= min_y) {
    camera_.camera_x = 0.5F * (min_x + max_x);
    camera_.camera_y = 0.5F * (min_y + max_y);

    const float kDx = max_x - min_x;
    const float kDy = max_y - min_y;
    const float kMaxDim = std::max(kDx, kDy);
    if (kMaxDim > 0.0F) {
      camera_.zoom = 300.0F / kMaxDim;
    } else {
      camera_.zoom = 10.0F;
    }
  }

  update();
}

auto ViewportWidget::FindActiveRoadType(const ast::Road& road, double s) -> ast::RoadType {
  if (road.types.empty()) {
    return ast::RoadType::kUnknown;
  }
  auto it = std::upper_bound(road.types.begin(), road.types.end(), s,
                             [](double val, const ast::RoadTypeRecord& record) -> bool { return val < record.s; });
  if (it == road.types.begin()) {
    return ast::RoadType::kUnknown;
  }
  return std::prev(it)->type;
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

  objects_vao_.create();
  objects_vbo_.create();
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
    SetupObjects();
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

  // 1. Draw Road Surface Meshes
  if (!geometry_.triangle_indices.empty()) {
    triangles_vao_.bind();
    for (const auto& range : geometry_.mesh_ranges) {
      if (!show_border_lanes_ &&
          (range.lane_type == ast::LaneType::kBorder || range.lane_type == ast::LaneType::kNone)) {
        continue;
      }
      if (range.index_count > 0) {
        const void* offset =
            reinterpret_cast<const void*>(static_cast<std::uintptr_t>(range.index_start) * sizeof(std::uint32_t));
        glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(range.index_count), GL_UNSIGNED_INT, offset);
      }
    }
    triangles_vao_.release();
  }

  // 2. Draw Boundaries/Markings in a Single batched call
  if (show_reference_lines_ && !geometry_.line_vertices.empty()) {
    glDisable(GL_DEPTH_TEST);
    lines_vao_.bind();
    glLineWidth(2.0F);
    glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(geometry_.line_vertices.size()));
    lines_vao_.release();
    glEnable(GL_DEPTH_TEST);
  }

  // Draw Objects in a Single batched call
  if (show_objects_ && !geometry_.object_line_vertices.empty()) {
    glDisable(GL_DEPTH_TEST);
    objects_vao_.bind();
    glLineWidth(2.0F);
    glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(geometry_.object_line_vertices.size()));
    objects_vao_.release();
    glEnable(GL_DEPTH_TEST);
  }

  // 3. Draw Hover Highlight Overlay
  if (has_model_ && hovered_pose_) {
    for (const auto& range : geometry_.mesh_ranges) {
      if (range.road_id == hovered_pose_->road && range.lane_id == hovered_pose_->lane) {
        if (!show_border_lanes_ &&
            (range.lane_type == ast::LaneType::kBorder || range.lane_type == ast::LaneType::kNone)) {
          break;
        }
        if (range.index_count > 0) {
          glDisable(GL_DEPTH_TEST);
          glEnable(GL_BLEND);
          glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

          shader_program_.setUniformValue("useOverrideColor", 1);
          shader_program_.setUniformValue("overrideColor", QVector4D(1.0F, 0.0F, 0.0F, 0.4F));

          triangles_vao_.bind();
          const void* offset =
              reinterpret_cast<const void*>(static_cast<std::uintptr_t>(range.index_start) * sizeof(std::uint32_t));
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

    if (has_model_) {
      // Draw dark glassmorphic card container in the top-left corner
      const QRect kRect(20, 20, 270, 204);
      painter.setPen(QPen(QColor(45, 51, 64, 255), 1));
      painter.setBrush(QBrush(QColor(26, 29, 36, 220)));
      painter.drawRoundedRect(kRect, 8.0, 8.0);

      // Setup font
      QFont font("Segoe UI", 10);
      painter.setFont(font);

      // Draw details
      const int kXOffset = 35;
      int y_offset = 45;
      const int kLineHeight = 22;

      // Header / Title
      font.setBold(true);
      painter.setFont(font);
      painter.setPen(QColor(255, 204, 0));  // Gold title color matching highlight
      painter.drawText(kXOffset, y_offset, "LANE INSPECTOR");

      font.setBold(false);
      painter.setFont(font);
      y_offset += kLineHeight;

      // Road ID
      painter.setPen(QColor(160, 170, 184));
      painter.drawText(kXOffset, y_offset, "Road ID:");
      painter.setPen(Qt::white);
      if (hovered_pose_) {
        painter.drawText(kXOffset + 70, y_offset, QString::fromStdString(hovered_road_name_));
      } else {
        painter.drawText(kXOffset + 70, y_offset, "--");
      }
      y_offset += kLineHeight;

      // Road Type
      painter.setPen(QColor(160, 170, 184));
      painter.drawText(kXOffset, y_offset, "Road Type:");
      painter.setPen(Qt::white);
      if (hovered_pose_) {
        ast::RoadType road_type = ast::RoadType::kUnknown;
        for (const auto& road : map_.roads) {
          if (road.id == hovered_road_name_) {
            road_type = FindActiveRoadType(road, hovered_pose_->s);
            break;
          }
        }
        painter.drawText(kXOffset + 80, y_offset, QString::fromStdString(std::string(parser::ToString(road_type))));
      } else {
        painter.drawText(kXOffset + 80, y_offset, "--");
      }
      y_offset += kLineHeight;

      // Lane ID
      painter.setPen(QColor(160, 170, 184));
      painter.drawText(kXOffset, y_offset, "Lane ID:");
      painter.setPen(Qt::white);
      if (hovered_pose_) {
        painter.drawText(kXOffset + 70, y_offset, QString::number(hovered_lane_original_id_));
      } else {
        painter.drawText(kXOffset + 70, y_offset, "--");
      }
      y_offset += kLineHeight;

      // Lane Type
      painter.setPen(QColor(160, 170, 184));
      painter.drawText(kXOffset, y_offset, "Lane Type:");
      painter.setPen(Qt::white);
      if (hovered_pose_) {
        ast::LaneType hovered_lane_type = ast::LaneType::kNone;
        for (const auto& range : geometry_.mesh_ranges) {
          if (range.road_id == hovered_pose_->road && range.lane_id == hovered_pose_->lane) {
            hovered_lane_type = range.lane_type;
            break;
          }
        }
        painter.drawText(kXOffset + 80, y_offset,
                         QString::fromStdString(std::string(parser::ToString(hovered_lane_type))));
      } else {
        painter.drawText(kXOffset + 80, y_offset, "--");
      }
      y_offset += kLineHeight;

      if (hovered_pose_) {
        // Obtain road-level and inertial-level coordinates from LanePose
        cpm::QueryContext temp_ctx;
        const cpm::RoadPose kRp = cpm_model_.LaneToRoad(*hovered_pose_, temp_ctx);
        const cpm::InertialPose kIp = cpm_model_.LaneToInertial(*hovered_pose_, temp_ctx);

        // Track Coordinates (s, t)
        painter.setPen(QColor(160, 170, 184));
        painter.drawText(kXOffset, y_offset, "Track (s, t):");
        painter.setPen(QColor(100, 181, 246));  // Light blue for values
        const QString kTrackCoords = QString("%1 m, %2 m").arg(kRp.s, 0, 'f', 3).arg(kRp.t, 0, 'f', 3);
        painter.drawText(kXOffset + 95, y_offset, kTrackCoords);
        y_offset += kLineHeight;

        // Lane Coordinates (s, t)
        painter.setPen(QColor(160, 170, 184));
        painter.drawText(kXOffset, y_offset, "Lane (s, t):");
        painter.setPen(QColor(100, 181, 246));  // Light blue for values
        const QString kLaneCoords =
            QString("%1 m, %2 m").arg(hovered_pose_->s, 0, 'f', 3).arg(hovered_pose_->t, 0, 'f', 3);
        painter.drawText(kXOffset + 95, y_offset, kLaneCoords);
        y_offset += kLineHeight;

        // Inertial Coordinates (x, y)
        painter.setPen(QColor(160, 170, 184));
        painter.drawText(kXOffset, y_offset, "Inertial (x, y):");
        painter.setPen(QColor(100, 181, 246));  // Light blue for values
        const QString kInertialCoords = QString("%1 m, %2 m").arg(kIp.x, 0, 'f', 3).arg(kIp.y, 0, 'f', 3);
        painter.drawText(kXOffset + 95, y_offset, kInertialCoords);
      } else {
        // Track Coordinates (s, t)
        painter.setPen(QColor(160, 170, 184));
        painter.drawText(kXOffset, y_offset, "Track (s, t):");
        painter.setPen(QColor(100, 181, 246));  // Light blue for values
        painter.drawText(kXOffset + 95, y_offset, "--");
        y_offset += kLineHeight;

        // Lane Coordinates (s, t)
        painter.setPen(QColor(160, 170, 184));
        painter.drawText(kXOffset, y_offset, "Lane (s, t):");
        painter.setPen(QColor(100, 181, 246));  // Light blue for values
        painter.drawText(kXOffset + 95, y_offset, "--");
        y_offset += kLineHeight;

        // Inertial Coordinates (x, y)
        painter.setPen(QColor(160, 170, 184));
        painter.drawText(kXOffset, y_offset, "Inertial (x, y):");
        painter.setPen(QColor(100, 181, 246));  // Light blue for values
        const QString kInertialCoords = QString("%1 m, %2 m").arg(hovered_x_, 0, 'f', 3).arg(hovered_y_, 0, 'f', 3);
        painter.drawText(kXOffset + 95, y_offset, kInertialCoords);
      }
    }

    // 5. Draw Compass Gizmo in the top-right corner
    {
      const int kCx = width() - 50;
      const int kCy = 50;

      // Draw dark glassmorphic circular background
      painter.setPen(QPen(QColor(45, 51, 64, 255), 1));
      painter.setBrush(QBrush(QColor(26, 29, 36, 220)));
      painter.drawEllipse(QPoint(kCx, kCy), 28, 28);

      // Save painter state to apply rotation
      painter.save();
      painter.translate(kCx, kCy);
      painter.rotate(-camera_.rotation);

      // Draw East axis (positive X) - Slate Blue / Premium Cyan
      painter.setPen(QPen(QColor(100, 181, 246, 255), 2));
      painter.drawLine(0, 0, 20, 0);

      // Draw North axis (positive Y, which is up, so -20 in QPainter screen Y) - Red / Premium Coral
      painter.setPen(QPen(QColor(255, 110, 110, 255), 2));
      painter.drawLine(0, 0, 0, -20);

      // Draw Labels E and N
      const QFont kFont("Segoe UI", 9, QFont::Bold);
      painter.setFont(kFont);

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
      const double kScaleLength = CalculateScaleLength(camera_.zoom);
      const double kS = kScaleLength * camera_.zoom;  // Width on screen

      const int kNumSegments = 4;
      const double kSegW = kS / kNumSegments;
      const double kX0 = width() - 20.0 - kS;
      for (int i = 0; i < kNumSegments; ++i) {
        const QRectF kSegRect(kX0 + (i * kSegW), height() - 35, kSegW, 8);
        if (i % 2 == 0) {
          painter.setBrush(QBrush(QColor(26, 29, 36)));  // Filled dark
        } else {
          painter.setBrush(QBrush(QColor(240, 240, 240)));  // Light segment
        }
        painter.setPen(QPen(QColor(240, 240, 240), 1));  // White border for contrast
        painter.drawRect(kSegRect);
      }

      // Draw text label centered above the scale bar
      painter.setPen(QColor(240, 240, 240));
      const QFont kFont("Segoe UI", 9, QFont::Bold);
      painter.setFont(kFont);
      QString label;
      if (kScaleLength >= 1000.0) {
        label = QString("%1 km").arg(kScaleLength / 1000.0);
      } else {
        label = QString("%1 m").arg(kScaleLength);
      }
      painter.drawText(QRectF(kX0, height() - 55, kS, 15), Qt::AlignCenter, label);
    }

    // 7. Draw Keyboard Shortcuts Panel in the bottom-left corner
    {
      const QRect kRect(20, height() - 210, 310, 190);
      painter.setPen(QPen(QColor(45, 51, 64, 255), 1));
      painter.setBrush(QBrush(QColor(26, 29, 36, 220)));
      painter.drawRoundedRect(kRect, 8.0, 8.0);

      // Setup font
      QFont font("Segoe UI", 9);
      painter.setFont(font);

      const int kXOffset = 35;
      int y_offset = height() - 185;
      const int kLineHeight = 20;

      // Header
      font.setBold(true);
      painter.setFont(font);
      painter.setPen(QColor(245, 197, 61));  // Amber title color
      painter.drawText(kXOffset, y_offset, "CONTROLS & SHORTCUTS");
      y_offset += 22;

      font.setBold(false);
      painter.setFont(font);

      struct ShortcutItem {
        QString key;
        QString desc;
      };
      const std::vector<ShortcutItem> kItems = {{"L-Click + Drag", "Pan Map"},  {"R-Click + Drag", "Rotate Map"},
                                                {"Scroll Wheel", "Zoom Map"},   {"Ctrl+R", "Reset View / Auto-fit"},
                                                {"R", "Toggle Reference Line"}, {"J", "Toggle Junction Boundaries"},
                                                {"B", "Toggle Border Lanes"}};

      for (const auto& item : kItems) {
        // Shortcut key
        painter.setPen(QColor(100, 181, 246));  // Light blue/cyan for keys
        painter.drawText(kXOffset, y_offset, item.key);

        // Description
        painter.setPen(QColor(180, 188, 204));  // Slate white for description
        painter.drawText(kXOffset + 95, y_offset, item.desc);

        y_offset += kLineHeight;
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

void ViewportWidget::SetupObjects() {
  objects_vao_.bind();

  // Load object line vertices
  objects_vbo_.bind();
  objects_vbo_.allocate(geometry_.object_line_vertices.data(),
                        static_cast<int>(geometry_.object_line_vertices.size() * sizeof(Vertex)));

  // Set attribute locations
  shader_program_.enableAttributeArray(0);
  shader_program_.setAttributeBuffer(0, GL_FLOAT, offsetof(Vertex, x), 3, sizeof(Vertex));

  shader_program_.enableAttributeArray(1);
  shader_program_.setAttributeBuffer(1, GL_FLOAT, offsetof(Vertex, r), 3, sizeof(Vertex));

  objects_vao_.release();
}

void ViewportWidget::mousePressEvent(QMouseEvent* event) {
  last_mouse_pos_ = event->pos();
  setFocus();
}

void ViewportWidget::mouseMoveEvent(QMouseEvent* event) {
  const QPoint kDelta = event->pos() - last_mouse_pos_;
  last_mouse_pos_ = event->pos();

  if ((event->buttons() & Qt::LeftButton) != 0) {
    camera_.Pan(static_cast<float>(kDelta.x()), static_cast<float>(kDelta.y()));
  } else if ((event->buttons() & Qt::RightButton) != 0) {
    camera_.Rotate(static_cast<float>(kDelta.x()) * 0.5F);
  }

  // Hover picking detection (CPU-side via CPM)
  if (has_model_) {
    const QPointF kWorldPos =
        camera_.ScreenToWorld(static_cast<float>(event->position().x()), static_cast<float>(event->position().y()));

    hovered_x_ = kWorldPos.x();
    hovered_y_ = kWorldPos.y();

    cpm::InertialPose pose{};
    pose.x = hovered_x_;
    pose.y = hovered_y_;

    // Query closest vertex in rendering geometry to find road elevation at mouse cursor
    float best_z = 0.0F;
    if (!geometry_.triangle_vertices.empty()) {
      float min_dist_sq = std::numeric_limits<float>::max();
      for (const auto& v : geometry_.triangle_vertices) {
        const float kDx = static_cast<float>(kWorldPos.x()) - v.x;
        const float kDy = static_cast<float>(kWorldPos.y()) - v.y;
        const float kDistSq = (kDx * kDx) + (kDy * kDy);
        if (kDistSq < min_dist_sq) {
          min_dist_sq = kDistSq;
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
    const QPoint kDelta = event->pixelDelta();
    camera_.Pan(static_cast<float>(kDelta.x()), static_cast<float>(kDelta.y()));
    update();
    return;
  }

  // Otherwise, it is a zoom event (mouse wheel or touchpad pinch Ctrl+scroll)
  const QPoint kNumPixels = event->pixelDelta();
  const QPoint kNumDegrees = event->angleDelta() / 8;

  float factor = 1.0F;
  if (!kNumPixels.isNull()) {
    factor = std::pow(1.15F, static_cast<float>(kNumPixels.y()) * 0.05F);
  } else if (!kNumDegrees.isNull()) {
    factor = std::pow(1.15F, static_cast<float>(kNumDegrees.y()) / 15.0F);
  }

  camera_.ZoomAt(static_cast<float>(event->position().x()), static_cast<float>(event->position().y()), factor);
  update();
}

void ViewportWidget::keyPressEvent(QKeyEvent* event) {
  if (event->key() == Qt::Key_R) {
    if (event->modifiers() & Qt::ControlModifier) {
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
      for (const auto& v : geometry_.object_line_vertices) {
        min_x = std::min(min_x, v.x);
        max_x = std::max(max_x, v.x);
        min_y = std::min(min_y, v.y);
        max_y = std::max(max_y, v.y);
      }
      if (max_x >= min_x && max_y >= min_y) {
        camera_.camera_x = 0.5F * (min_x + max_x);
        camera_.camera_y = 0.5F * (min_y + max_y);
        const float kDx = max_x - min_x;
        const float kDy = max_y - min_y;
        const float kMaxDim = std::max(kDx, kDy);
        if (kMaxDim > 0.0F) {
          camera_.zoom = 300.0F / kMaxDim;
        }
      }
      update();
    } else {
      show_reference_lines_ = !show_reference_lines_;
      update();
    }
  } else if (event->key() == Qt::Key_J) {
    show_junction_boundaries_ = !show_junction_boundaries_;
    update();
  } else if (event->key() == Qt::Key_B) {
    show_border_lanes_ = !show_border_lanes_;
    update();
  } else if (event->key() == Qt::Key_O) {
    show_objects_ = !show_objects_;
    update();
    if (auto* main_win = qobject_cast<QMainWindow*>(window())) {
      if (auto* status = main_win->statusBar()) {
        status->showMessage(show_objects_ ? "Toggled objects: ON" : "Toggled objects: OFF", 2000);
      }
    }
  }
}

auto ViewportWidget::event(QEvent* event) -> bool {
  if (event->type() == QEvent::NativeGesture) {
    auto* gesture_event = dynamic_cast<QNativeGestureEvent*>(event);
    if (gesture_event->gestureType() == Qt::ZoomNativeGesture) {
      const qreal kVal = gesture_event->value();
      const QPointF kPos = gesture_event->position();
      camera_.ZoomAt(static_cast<float>(kPos.x()), static_cast<float>(kPos.y()), static_cast<float>(1.0 + kVal));
      update();
      return true;
    }
    if (gesture_event->gestureType() == Qt::RotateNativeGesture) {
      const qreal kVal = gesture_event->value();
      camera_.Rotate(-static_cast<float>(kVal));
      update();
      return true;
    }
  }
  return QOpenGLWidget::event(event);
}

void ViewportWidget::RenderGrid() {
  const double kScaleLength = CalculateScaleLength(camera_.zoom);
  if (kScaleLength <= 0.0) {
    return;
  }

  // Calculate maximum visible bounds in world coordinates
  auto w = static_cast<float>(width());
  auto h = static_cast<float>(height());
  const float kRScreen = std::sqrt((w * w) + (h * h)) / 2.0F;
  const float kRWorld = kRScreen / camera_.zoom;

  const float kMinX = camera_.camera_x - kRWorld;
  const float kMaxX = camera_.camera_x + kRWorld;
  const float kMinY = camera_.camera_y - kRWorld;
  const float kMaxY = camera_.camera_y + kRWorld;

  auto grid_spacing = static_cast<float>(kScaleLength);

  std::vector<Vertex> grid_vertices;

  // Grid line color: subtle dark grey-blue
  const float kR = 0.16F;
  const float kG = 0.18F;
  const float kB = 0.22F;

  // Vertical lines (constant x)
  const float kStartX = std::floor(kMinX / grid_spacing) * grid_spacing;
  const float kEndX = std::ceil(kMaxX / grid_spacing) * grid_spacing;
  for (float x = kStartX; x <= kEndX; x += grid_spacing) {
    const bool is_axis = std::abs(x) < 1e-4F;
    const float r = is_axis ? 0.35F : kR;
    const float g = is_axis ? 0.40F : kG;
    const float b = is_axis ? 0.50F : kB;
    grid_vertices.push_back(Vertex{.x = x, .y = kMinY, .z = 0.0F, .r = r, .g = g, .b = b});
    grid_vertices.push_back(Vertex{.x = x, .y = kMaxY, .z = 0.0F, .r = r, .g = g, .b = b});
  }

  // Horizontal lines (constant y)
  const float kStartY = std::floor(kMinY / grid_spacing) * grid_spacing;
  const float kEndY = std::ceil(kMaxY / grid_spacing) * grid_spacing;
  for (float y = kStartY; y <= kEndY; y += grid_spacing) {
    const bool is_axis = std::abs(y) < 1e-4F;
    const float r = is_axis ? 0.35F : kR;
    const float g = is_axis ? 0.40F : kG;
    const float b = is_axis ? 0.50F : kB;
    grid_vertices.push_back(Vertex{.x = kMinX, .y = y, .z = 0.0F, .r = r, .g = g, .b = b});
    grid_vertices.push_back(Vertex{.x = kMaxX, .y = y, .z = 0.0F, .r = r, .g = g, .b = b});
  }

  // Mark the origin (0, 0) with a prominent crosshair
  if (kMinX <= 0.0F && 0.0F <= kMaxX && kMinY <= 0.0F && 0.0F <= kMaxY) {
    const float kOriginR = 1.0F;
    const float kOriginG = 0.6F;
    const float kOriginB = 0.0F;
    const float kCrossHalf = 0.15F * grid_spacing;

    // Vertical segment
    grid_vertices.push_back(
        Vertex{.x = 0.0F, .y = -kCrossHalf, .z = 0.0F, .r = kOriginR, .g = kOriginG, .b = kOriginB});
    grid_vertices.push_back(Vertex{.x = 0.0F, .y = kCrossHalf, .z = 0.0F, .r = kOriginR, .g = kOriginG, .b = kOriginB});
    // Horizontal segment
    grid_vertices.push_back(
        Vertex{.x = -kCrossHalf, .y = 0.0F, .z = 0.0F, .r = kOriginR, .g = kOriginG, .b = kOriginB});
    grid_vertices.push_back(Vertex{.x = kCrossHalf, .y = 0.0F, .z = 0.0F, .r = kOriginR, .g = kOriginG, .b = kOriginB});
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
