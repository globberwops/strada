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
#include <strada/vis/colors.hpp>
#include <strada/vis/viewport_widget.hpp>

namespace {

auto ToQColor(strada::vis::Color c) noexcept -> QColor { return QColor::fromRgbF(c.r, c.g, c.b, 1.0F); }

auto ToQColor(strada::vis::ColorA c) noexcept -> QColor { return QColor::fromRgbF(c.r, c.g, c.b, c.a); }

}  // namespace

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
  signals_vao_.destroy();
  signals_vbo_.destroy();
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

  for (const auto& v : geometry_.signal_line_vertices) {
    min_x = std::min(min_x, v.x);
    max_x = std::max(max_x, v.x);
    min_y = std::min(min_y, v.y);
    max_y = std::max(max_y, v.y);
  }

  if (max_x >= min_x && max_y >= min_y) {
    camera_.camera_x = 0.5F * (min_x + max_x);
    camera_.camera_y = 0.5F * (min_y + max_y);

    const float dx = max_x - min_x;
    const float dy = max_y - min_y;
    const float max_dim = std::max(dx, dy);
    if (max_dim > 0.0F) {
      camera_.zoom = 300.0F / max_dim;
    } else {
      camera_.zoom = 10.0F;
    }
  }

  routing_graph_.emplace(map_);
  waypoint_road_ids_.clear();
  waypoint_world_coords_.clear();
  active_route_ = std::nullopt;
  route_error_.clear();

  update();
}

auto ViewportWidget::IsRouteCreationMode() const -> bool { return route_creation_mode_; }

auto ViewportWidget::Waypoints() const -> const std::vector<std::string>& { return waypoint_road_ids_; }

auto ViewportWidget::WaypointCoords() const -> const std::vector<QPointF>& { return waypoint_world_coords_; }

auto ViewportWidget::ActiveRoute() const -> const std::optional<routing::Route>& { return active_route_; }

auto ViewportWidget::GetCamera() const -> const Camera& { return camera_; }

auto ViewportWidget::RouteError() const -> std::string { return route_error_; }

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

  signals_vao_.create();
  signals_vbo_.create();
}

void ViewportWidget::resizeGL(int w, int h) {
  glViewport(0, 0, w, h);
  camera_.SetViewport(w, h);
}

void ViewportWidget::paintGL() {
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  if (geometry_dirty_) {
    UpdateGeometryBuffers();
    geometry_dirty_ = false;
  }

  DrawScene();
  DrawOverlays();
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

void ViewportWidget::SetupSignals() {
  signals_vao_.bind();

  // Load signal line vertices
  signals_vbo_.bind();
  signals_vbo_.allocate(geometry_.signal_line_vertices.data(),
                        static_cast<int>(geometry_.signal_line_vertices.size() * sizeof(Vertex)));

  // Set attribute locations
  shader_program_.enableAttributeArray(0);
  shader_program_.setAttributeBuffer(0, GL_FLOAT, offsetof(Vertex, x), 3, sizeof(Vertex));

  shader_program_.enableAttributeArray(1);
  shader_program_.setAttributeBuffer(1, GL_FLOAT, offsetof(Vertex, r), 3, sizeof(Vertex));

  signals_vao_.release();
}

void ViewportWidget::mousePressEvent(QMouseEvent* event) {
  last_mouse_pos_ = event->pos();
  mouse_press_pos_ = event->pos();
  setFocus();
}

void ViewportWidget::mouseReleaseEvent(QMouseEvent* event) {
  if (event->button() == Qt::LeftButton) {
    const auto dx = static_cast<double>(event->pos().x() - mouse_press_pos_.x());
    const auto dy = static_cast<double>(event->pos().y() - mouse_press_pos_.y());
    const auto distance = std::sqrt((dx * dx) + (dy * dy));

    if (distance <= 5.0) {
      if (route_creation_mode_ && has_model_) {
        const auto world_pos =
            camera_.ScreenToWorld(static_cast<float>(event->position().x()), static_cast<float>(event->position().y()));

        auto pose = cpm::InertialPose{};
        pose.x = world_pos.x();
        pose.y = world_pos.y();

        auto best_z = 0.0F;
        if (!geometry_.triangle_vertices.empty()) {
          auto min_dist_sq = std::numeric_limits<float>::max();
          for (const auto& v : geometry_.triangle_vertices) {
            const auto dx_v = static_cast<float>(world_pos.x()) - v.x;
            const auto dy_v = static_cast<float>(world_pos.y()) - v.y;
            const auto dist_sq = (dx_v * dx_v) + (dy_v * dy_v);
            if (dist_sq < min_dist_sq) {
              min_dist_sq = dist_sq;
              best_z = v.z;
            }
          }
        }
        pose.z = static_cast<double>(best_z);
        pose.heading = 0.0;
        pose.pitch = 0.0;
        pose.roll = 0.0;

        const auto lp_opt = cpm_model_.InertialToLane(pose, query_ctx_);
        if (lp_opt.has_value()) {
          if (IsDrivableLane(lp_opt->road, lp_opt->lane)) {
            const auto road_id_str = std::string(cpm_model_.OriginalRoadId(lp_opt->road));
            waypoint_road_ids_.push_back(road_id_str);

            auto snapped_pose = *lp_opt;
            snapped_pose.t = 0.0;
            cpm::QueryContext temp_ctx;
            const auto snapped_ip = cpm_model_.LaneToInertial(snapped_pose, temp_ctx);
            waypoint_world_coords_.push_back(QPointF{snapped_ip.x, snapped_ip.y});

            RecomputeRoute();
            update();
          }
        }
      }
    }
  }
}

void ViewportWidget::mouseMoveEvent(QMouseEvent* event) {
  const QPoint delta = event->pos() - last_mouse_pos_;
  last_mouse_pos_ = event->pos();

  if ((event->buttons() & Qt::LeftButton) != 0) {
    camera_.Pan(static_cast<float>(delta.x()), static_cast<float>(delta.y()));
  } else if ((event->buttons() & Qt::RightButton) != 0) {
    camera_.Rotate(static_cast<float>(delta.x()) * 0.5F);
  }

  // Hover picking detection (CPU-side via CPM)
  if (has_model_) {
    const QPointF world_pos =
        camera_.ScreenToWorld(static_cast<float>(event->position().x()), static_cast<float>(event->position().y()));

    hovered_x_ = world_pos.x();
    hovered_y_ = world_pos.y();

    cpm::InertialPose pose{};
    pose.x = hovered_x_;
    pose.y = hovered_y_;

    // Query closest vertex in rendering geometry to find road elevation at mouse cursor
    float best_z = 0.0F;
    if (!geometry_.triangle_vertices.empty()) {
      float min_dist_sq = std::numeric_limits<float>::max();
      for (const auto& v : geometry_.triangle_vertices) {
        const float dx = static_cast<float>(world_pos.x()) - v.x;
        const float dy = static_cast<float>(world_pos.y()) - v.y;
        const float dist_sq = (dx * dx) + (dy * dy);
        if (dist_sq < min_dist_sq) {
          min_dist_sq = dist_sq;
          best_z = v.z;
        }
      }
    }
    pose.z = static_cast<double>(best_z);

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
    const QPoint delta = event->pixelDelta();
    camera_.Pan(static_cast<float>(delta.x()), static_cast<float>(delta.y()));
    update();
    return;
  }

  // Otherwise, it is a zoom event (mouse wheel or touchpad pinch Ctrl+scroll)
  const QPoint num_pixels = event->pixelDelta();
  const QPoint num_degrees = event->angleDelta() / 8;

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
        const float dx = max_x - min_x;
        const float dy = max_y - min_y;
        const float max_dim = std::max(dx, dy);
        if (max_dim > 0.0F) {
          camera_.zoom = 300.0F / max_dim;
        }
      }
      update();
    } else {
      show_reference_lines_ = !show_reference_lines_;
      update();
      if (auto* main_win = qobject_cast<QMainWindow*>(window())) {
        if (auto* status = main_win->statusBar()) {
          status->showMessage(show_reference_lines_ ? "Toggled reference lines: ON" : "Toggled reference lines: OFF",
                              2000);
        }
      }
    }
  } else if (event->key() == Qt::Key_J) {
    show_junction_boundaries_ = !show_junction_boundaries_;
    update();
    if (auto* main_win = qobject_cast<QMainWindow*>(window())) {
      if (auto* status = main_win->statusBar()) {
        status->showMessage(
            show_junction_boundaries_ ? "Toggled junction boundaries: ON" : "Toggled junction boundaries: OFF", 2000);
      }
    }
  } else if (event->key() == Qt::Key_B) {
    show_border_lanes_ = !show_border_lanes_;
    update();
    if (auto* main_win = qobject_cast<QMainWindow*>(window())) {
      if (auto* status = main_win->statusBar()) {
        status->showMessage(show_border_lanes_ ? "Toggled border lanes: ON" : "Toggled border lanes: OFF", 2000);
      }
    }
  } else if (event->key() == Qt::Key_O) {
    show_objects_ = !show_objects_;
    update();
    if (auto* main_win = qobject_cast<QMainWindow*>(window())) {
      if (auto* status = main_win->statusBar()) {
        status->showMessage(show_objects_ ? "Toggled objects: ON" : "Toggled objects: OFF", 2000);
      }
    }
  } else if (event->key() == Qt::Key_S) {
    show_signals_ = !show_signals_;
    update();
    if (auto* main_win = qobject_cast<QMainWindow*>(window())) {
      if (auto* status = main_win->statusBar()) {
        status->showMessage(show_signals_ ? "Toggled signals: ON" : "Toggled signals: OFF", 2000);
      }
    }
  } else if (event->key() == Qt::Key_L) {
    show_lanes_ = !show_lanes_;
    update();
    if (auto* main_win = qobject_cast<QMainWindow*>(window())) {
      if (auto* status = main_win->statusBar()) {
        status->showMessage(show_lanes_ ? "Toggled lanes: ON" : "Toggled lanes: OFF", 2000);
      }
    }
  } else if (event->key() == Qt::Key_P) {
    route_creation_mode_ = !route_creation_mode_;
    update();
    if (auto* main_win = qobject_cast<QMainWindow*>(window())) {
      if (auto* status = main_win->statusBar()) {
        status->showMessage(route_creation_mode_ ? "Route Creation Mode: ON" : "Route Creation Mode: OFF", 2000);
      }
    }
  } else if (route_creation_mode_) {
    if (event->key() == Qt::Key_Backspace || event->key() == Qt::Key_Delete) {
      if (!waypoint_road_ids_.empty()) {
        waypoint_road_ids_.pop_back();
        if (!waypoint_world_coords_.empty()) {
          waypoint_world_coords_.pop_back();
        }
        RecomputeRoute();
        update();
        if (auto* main_win = qobject_cast<QMainWindow*>(window())) {
          if (auto* status = main_win->statusBar()) {
            status->showMessage("Undone last waypoint", 2000);
          }
        }
      }
    } else if (event->key() == Qt::Key_C) {
      waypoint_road_ids_.clear();
      waypoint_world_coords_.clear();
      active_route_ = std::nullopt;
      route_error_.clear();
      update();
      if (auto* main_win = qobject_cast<QMainWindow*>(window())) {
        if (auto* status = main_win->statusBar()) {
          status->showMessage("Cleared route and waypoints", 2000);
        }
      }
    } else if (event->key() == Qt::Key_Escape) {
      route_creation_mode_ = false;
      route_error_.clear();
      update();
      if (auto* main_win = qobject_cast<QMainWindow*>(window())) {
        if (auto* status = main_win->statusBar()) {
          status->showMessage("Route Creation Mode: OFF", 2000);
        }
      }
    }
  }
}

auto ViewportWidget::event(QEvent* event) -> bool {
  if (event->type() == QEvent::NativeGesture) {
    auto* gesture_event = dynamic_cast<QNativeGestureEvent*>(event);
    if (gesture_event->gestureType() == Qt::ZoomNativeGesture) {
      const qreal val = gesture_event->value();
      const QPointF pos = gesture_event->position();
      camera_.ZoomAt(static_cast<float>(pos.x()), static_cast<float>(pos.y()), static_cast<float>(1.0 + val));
      update();
      return true;
    }
    if (gesture_event->gestureType() == Qt::RotateNativeGesture) {
      const qreal val = gesture_event->value();
      camera_.Rotate(-static_cast<float>(val));
      update();
      return true;
    }
  }
  return QOpenGLWidget::event(event);
}

void ViewportWidget::RenderGrid() {
  const double scale_length = CalculateScaleLength(static_cast<double>(camera_.zoom));
  if (scale_length <= 0.0) {
    return;
  }

  // Calculate maximum visible bounds in world coordinates
  auto w = static_cast<float>(width());
  auto h = static_cast<float>(height());
  const float r_screen = std::sqrt((w * w) + (h * h)) / 2.0F;
  const float r_world = r_screen / camera_.zoom;

  const float min_x = camera_.camera_x - r_world;
  const float max_x = camera_.camera_x + r_world;
  const float min_y = camera_.camera_y - r_world;
  const float max_y = camera_.camera_y + r_world;

  auto grid_spacing = static_cast<float>(scale_length);

  std::vector<Vertex> grid_vertices;

  // Grid line color: subtle dark grey-blue
  constexpr auto base_r = 0.16F;
  constexpr auto base_g = 0.18F;
  constexpr auto base_b = 0.22F;

  // Vertical lines (constant x)
  const float start_x = std::floor(min_x / grid_spacing) * grid_spacing;
  const float end_x = std::ceil(max_x / grid_spacing) * grid_spacing;
  for (float x = start_x; x <= end_x; x += grid_spacing) {
    const bool is_axis = std::abs(x) < 1e-4F;
    const auto r = is_axis ? 0.35F : base_r;
    const auto g = is_axis ? 0.40F : base_g;
    const auto b = is_axis ? 0.50F : base_b;
    grid_vertices.push_back(Vertex{.x = x, .y = min_y, .z = 0.0F, .r = r, .g = g, .b = b});
    grid_vertices.push_back(Vertex{.x = x, .y = max_y, .z = 0.0F, .r = r, .g = g, .b = b});
  }

  // Horizontal lines (constant y)
  const float start_y = std::floor(min_y / grid_spacing) * grid_spacing;
  const float end_y = std::ceil(max_y / grid_spacing) * grid_spacing;
  for (float y = start_y; y <= end_y; y += grid_spacing) {
    const bool is_axis = std::abs(y) < 1e-4F;
    const auto r = is_axis ? 0.35F : base_r;
    const auto g = is_axis ? 0.40F : base_g;
    const auto b = is_axis ? 0.50F : base_b;
    grid_vertices.push_back(Vertex{.x = min_x, .y = y, .z = 0.0F, .r = r, .g = g, .b = b});
    grid_vertices.push_back(Vertex{.x = max_x, .y = y, .z = 0.0F, .r = r, .g = g, .b = b});
  }

  // Mark the origin (0, 0) with a prominent crosshair
  if (min_x <= 0.0F && 0.0F <= max_x && min_y <= 0.0F && 0.0F <= max_y) {
    const float origin_r = 1.0F;
    const float origin_g = 0.6F;
    const float origin_b = 0.0F;
    const float cross_half = 0.15F * grid_spacing;

    // Vertical segment
    grid_vertices.push_back(
        Vertex{.x = 0.0F, .y = -cross_half, .z = 0.0F, .r = origin_r, .g = origin_g, .b = origin_b});
    grid_vertices.push_back(Vertex{.x = 0.0F, .y = cross_half, .z = 0.0F, .r = origin_r, .g = origin_g, .b = origin_b});
    // Horizontal segment
    grid_vertices.push_back(
        Vertex{.x = -cross_half, .y = 0.0F, .z = 0.0F, .r = origin_r, .g = origin_g, .b = origin_b});
    grid_vertices.push_back(Vertex{.x = cross_half, .y = 0.0F, .z = 0.0F, .r = origin_r, .g = origin_g, .b = origin_b});
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

void ViewportWidget::UpdateGeometryBuffers() {
  SetupTriangles();
  SetupLines();
  SetupBoundaries();
  SetupObjects();
  SetupSignals();
}

void ViewportWidget::DrawScene() {
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
    shader_program_.setUniformValue("overrideColor", QVector4D{kJunctionHighlight.r, kJunctionHighlight.g,
                                                               kJunctionHighlight.b, kJunctionHighlight.a});
    boundaries_vao_.bind();
    glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(geometry_.boundary_triangle_indices.size()), GL_UNSIGNED_INT,
                   nullptr);
    boundaries_vao_.release();
    shader_program_.setUniformValue("useOverrideColor", 0);
    glDisable(GL_BLEND);
  }

  // 1. Draw Road Surface Meshes
  if (show_lanes_ && !geometry_.triangle_indices.empty()) {
    triangles_vao_.bind();
    for (const auto& range : geometry_.mesh_ranges) {
      if (!show_border_lanes_ &&
          (range.lane_type == ast::LaneType::kBorder || range.lane_type == ast::LaneType::kNone)) {
        continue;
      }
      if (range.index_count > 0) {
        const auto* offset = reinterpret_cast<const void*>(  // NOLINT(performance-no-int-to-ptr,
                                                             // cppcoreguidelines-pro-type-reinterpret-cast)
            static_cast<std::uintptr_t>(range.index_start) * sizeof(std::uint32_t));
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

  // Draw Signals in a Single batched call
  if (show_signals_ && !geometry_.signal_line_vertices.empty()) {
    glDisable(GL_DEPTH_TEST);
    signals_vao_.bind();
    glLineWidth(2.0F);
    glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(geometry_.signal_line_vertices.size()));
    signals_vao_.release();
    glEnable(GL_DEPTH_TEST);
  }

  // 3. Draw Hover Highlight Overlay
  if (show_lanes_ && has_model_ && hovered_pose_) {
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
          shader_program_.setUniformValue(
              "overrideColor", QVector4D{kHoverHighlight.r, kHoverHighlight.g, kHoverHighlight.b, kHoverHighlight.a});

          triangles_vao_.bind();
          const auto* offset = reinterpret_cast<const void*>(  // NOLINT(performance-no-int-to-ptr,
                                                               // cppcoreguidelines-pro-type-reinterpret-cast)
              static_cast<std::uintptr_t>(range.index_start) * sizeof(std::uint32_t));
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

  // 4. Draw Route Highlight Overlay
  if (show_lanes_ && has_model_ && active_route_.has_value() && !active_route_->segments.empty()) {
    glDisable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    shader_program_.setUniformValue("useOverrideColor", 1);
    shader_program_.setUniformValue("overrideColor", QVector4D{0.0F, 229.0F / 255.0F, 1.0F, 0.4F});

    triangles_vao_.bind();
    for (const auto& range : geometry_.mesh_ranges) {
      bool in_route = false;
      const auto original_id = cpm_model_.OriginalRoadId(range.road_id);
      for (const auto& seg : active_route_->segments) {
        if (seg.road_id == original_id) {
          in_route = true;
          break;
        }
      }

      if (in_route && (range.lane_type == ast::LaneType::kDriving || range.lane_type == ast::LaneType::kOnRamp ||
                       range.lane_type == ast::LaneType::kExit || range.lane_type == ast::LaneType::kEntry ||
                       range.lane_type == ast::LaneType::kConnectingRamp)) {
        if (range.index_count > 0) {
          const auto* offset =
              reinterpret_cast<const void*>(static_cast<std::uintptr_t>(range.index_start) * sizeof(std::uint32_t));
          glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(range.index_count), GL_UNSIGNED_INT, offset);
        }
      }
    }
    triangles_vao_.release();
    shader_program_.setUniformValue("useOverrideColor", 0);
    glDisable(GL_BLEND);
    glEnable(GL_DEPTH_TEST);
  }

  shader_program_.release();
}

void ViewportWidget::DrawLaneInspector(QPainter& painter) {
  if (!has_model_) {
    return;
  }

  // Draw dark glassmorphic card container in the top-left corner
  const auto rect = QRect{20, 20, 270, 226};
  painter.setPen(QPen(ToQColor(kUIBorder), 1));
  painter.setBrush(QBrush(ToQColor(kUIBackground)));
  painter.drawRoundedRect(rect, 8.0, 8.0);

  // Setup font
  auto font = QFont{"Segoe UI", 10};
  painter.setFont(font);

  // Draw details
  const auto x_offset = 35;
  auto y_offset = 45;
  const auto line_height = 22;

  // Header / Title
  font.setBold(true);
  painter.setFont(font);
  painter.setPen(ToQColor(kTextGold));  // Gold title color matching highlight
  painter.drawText(x_offset, y_offset, "LANE INSPECTOR");

  font.setBold(false);
  painter.setFont(font);
  y_offset += line_height;

  // Road ID
  painter.setPen(ToQColor(kTextLabel));
  painter.drawText(x_offset, y_offset, "Road ID:");
  painter.setPen(Qt::white);
  if (hovered_pose_) {
    painter.drawText(x_offset + 70, y_offset, QString::fromStdString(hovered_road_name_));
  } else {
    painter.drawText(x_offset + 70, y_offset, "--");
  }
  y_offset += line_height;

  // Road Type
  painter.setPen(ToQColor(kTextLabel));
  painter.drawText(x_offset, y_offset, "Road Type:");
  painter.setPen(Qt::white);
  if (hovered_pose_) {
    auto road_type = ast::RoadType::kUnknown;
    for (const auto& road : map_.roads) {
      if (road.id == hovered_road_name_) {
        road_type = FindActiveRoadType(road, hovered_pose_->s);
        break;
      }
    }
    painter.drawText(x_offset + 80, y_offset, QString::fromStdString(std::string(parser::ToString(road_type))));
  } else {
    painter.drawText(x_offset + 80, y_offset, "--");
  }
  y_offset += line_height;

  // Lane ID
  painter.setPen(ToQColor(kTextLabel));
  painter.drawText(x_offset, y_offset, "Lane ID:");
  painter.setPen(Qt::white);
  if (hovered_pose_) {
    painter.drawText(x_offset + 70, y_offset, QString::number(hovered_lane_original_id_));
  } else {
    painter.drawText(x_offset + 70, y_offset, "--");
  }
  y_offset += line_height;

  // Lane Type
  painter.setPen(ToQColor(kTextLabel));
  painter.drawText(x_offset, y_offset, "Lane Type:");
  painter.setPen(Qt::white);
  if (hovered_pose_) {
    auto hovered_lane_type = ast::LaneType::kNone;
    for (const auto& range : geometry_.mesh_ranges) {
      if (range.road_id == hovered_pose_->road && range.lane_id == hovered_pose_->lane) {
        hovered_lane_type = range.lane_type;
        break;
      }
    }
    painter.drawText(x_offset + 80, y_offset, QString::fromStdString(std::string(parser::ToString(hovered_lane_type))));
  } else {
    painter.drawText(x_offset + 80, y_offset, "--");
  }
  y_offset += line_height;

  if (hovered_pose_) {
    // Obtain road-level and inertial-level coordinates from LanePose
    cpm::QueryContext temp_ctx;
    const auto rp = cpm_model_.LaneToRoad(*hovered_pose_, temp_ctx);
    const auto ip = cpm_model_.LaneToInertial(*hovered_pose_, temp_ctx);

    // Track Coordinates (s, t)
    painter.setPen(ToQColor(kTextLabel));
    painter.drawText(x_offset, y_offset, "Track (s, t):");
    painter.setPen(ToQColor(kTextValue));  // Light blue for values
    const auto track_coords = QString{"%1 m, %2 m"}.arg(rp.s, 0, 'f', 3).arg(rp.t, 0, 'f', 3);
    painter.drawText(x_offset + 95, y_offset, track_coords);
    y_offset += line_height;

    // Lane Coordinates (s, t)
    painter.setPen(ToQColor(kTextLabel));
    painter.drawText(x_offset, y_offset, "Lane (s, t):");
    painter.setPen(ToQColor(kTextValue));  // Light blue for values
    const auto lane_coords = QString{"%1 m, %2 m"}.arg(hovered_pose_->s, 0, 'f', 3).arg(hovered_pose_->t, 0, 'f', 3);
    painter.drawText(x_offset + 95, y_offset, lane_coords);
    y_offset += line_height;

    // Route Coordinates (s, t)
    painter.setPen(ToQColor(kTextLabel));
    painter.drawText(x_offset, y_offset, "Route (s, t):");
    painter.setPen(ToQColor(kTextValue));
    if (active_route_.has_value()) {
      const auto route_coords_opt = active_route_->ToRouteCoordinates(hovered_road_name_, rp.s, rp.t);
      if (route_coords_opt.has_value()) {
        const auto route_coords_str =
            QString{"%1 m, %2 m"}.arg(route_coords_opt->first, 0, 'f', 3).arg(route_coords_opt->second, 0, 'f', 3);
        painter.drawText(x_offset + 95, y_offset, route_coords_str);
      } else {
        painter.drawText(x_offset + 95, y_offset, "--");
      }
    } else {
      painter.drawText(x_offset + 95, y_offset, "--");
    }
    y_offset += line_height;

    // Inertial Coordinates (x, y)
    painter.setPen(ToQColor(kTextLabel));
    painter.drawText(x_offset, y_offset, "Inertial (x, y):");
    painter.setPen(ToQColor(kTextValue));  // Light blue for values
    const auto inertial_coords = QString{"%1 m, %2 m"}.arg(ip.x, 0, 'f', 3).arg(ip.y, 0, 'f', 3);
    painter.drawText(x_offset + 95, y_offset, inertial_coords);
  } else {
    // Track Coordinates (s, t)
    painter.setPen(ToQColor(kTextLabel));
    painter.drawText(x_offset, y_offset, "Track (s, t):");
    painter.setPen(ToQColor(kTextValue));  // Light blue for values
    painter.drawText(x_offset + 95, y_offset, "--");
    y_offset += line_height;

    // Lane Coordinates (s, t)
    painter.setPen(ToQColor(kTextLabel));
    painter.drawText(x_offset, y_offset, "Lane (s, t):");
    painter.setPen(ToQColor(kTextValue));  // Light blue for values
    painter.drawText(x_offset + 95, y_offset, "--");
    y_offset += line_height;

    // Route Coordinates (s, t)
    painter.setPen(ToQColor(kTextLabel));
    painter.drawText(x_offset, y_offset, "Route (s, t):");
    painter.setPen(ToQColor(kTextValue));  // Light blue for values
    painter.drawText(x_offset + 95, y_offset, "--");
    y_offset += line_height;

    // Inertial Coordinates (x, y)
    painter.setPen(ToQColor(kTextLabel));
    painter.drawText(x_offset, y_offset, "Inertial (x, y):");
    painter.setPen(ToQColor(kTextValue));  // Light blue for values
    const auto inertial_coords = QString{"%1 m, %2 m"}.arg(hovered_x_, 0, 'f', 3).arg(hovered_y_, 0, 'f', 3);
    painter.drawText(x_offset + 95, y_offset, inertial_coords);
  }
}

void ViewportWidget::DrawCompass(QPainter& painter) {
  const auto cx = width() - 50;
  const auto cy = 50;

  // Draw dark glassmorphic circular background
  painter.setPen(QPen(ToQColor(kUIBorder), 1));
  painter.setBrush(QBrush(ToQColor(kUIBackground)));
  painter.drawEllipse(QPoint(cx, cy), 28, 28);

  // Save painter state to apply rotation
  painter.save();
  painter.translate(cx, cy);
  painter.rotate(-static_cast<qreal>(camera_.rotation));

  // Draw East axis (positive X) - Slate Blue / Premium Cyan
  painter.setPen(QPen(ToQColor(kCompassEast), 2));
  painter.drawLine(0, 0, 20, 0);

  // Draw North axis (positive Y, which is up, so -20 in QPainter screen Y) - Red / Premium Coral
  painter.setPen(QPen(ToQColor(kCompassNorth), 2));
  painter.drawLine(0, 0, 0, -20);

  // Draw Labels E and N
  const auto font = QFont{"Segoe UI", 9, QFont::Bold};
  painter.setFont(font);

  // E Label
  painter.setPen(ToQColor(kCompassEast));
  painter.drawText(QRect{22, -8, 16, 16}, Qt::AlignCenter, "E");

  // N Label
  painter.setPen(ToQColor(kCompassNorth));
  painter.drawText(QRect{-8, -36, 16, 16}, Qt::AlignCenter, "N");

  painter.restore();
}

void ViewportWidget::DrawScaleBar(QPainter& painter) {
  const auto scale_length = CalculateScaleLength(static_cast<double>(camera_.zoom));
  const auto bar_width = scale_length * static_cast<double>(camera_.zoom);  // Width on screen

  const auto num_segments = 4;
  const auto seg_width = bar_width / static_cast<double>(num_segments);
  const auto bar_start_x = static_cast<double>(width()) - 20.0 - bar_width;
  for (auto i = 0; i < num_segments; ++i) {
    const auto seg_rect = QRectF{bar_start_x + (static_cast<double>(i) * seg_width),
                                 static_cast<double>(height()) - 35.0, seg_width, 8.0};
    if (i % 2 == 0) {
      painter.setBrush(QBrush(ToQColor(kUIBackgroundOpaque)));  // Filled dark
    } else {
      painter.setBrush(QBrush(ToQColor(kTextLight)));  // Light segment
    }
    painter.setPen(QPen(ToQColor(kTextLight), 1));  // White border for contrast
    painter.drawRect(seg_rect);
  }

  // Draw text label centered above the scale bar
  painter.setPen(ToQColor(kTextLight));
  const auto font = QFont{"Segoe UI", 9, QFont::Bold};
  painter.setFont(font);
  auto label = QString{};
  if (scale_length >= 1000.0) {
    label = QString{"%1 km"}.arg(scale_length / 1000.0);
  } else {
    label = QString{"%1 m"}.arg(scale_length);
  }
  painter.drawText(QRectF{bar_start_x, static_cast<double>(height()) - 55.0, bar_width, 15.0}, Qt::AlignCenter, label);
}

void ViewportWidget::DrawShortcutsPanel(QPainter& painter) {
  const auto rect = QRect{20, height() - 270, 310, 250};
  painter.setPen(QPen(ToQColor(kUIBorder), 1));
  painter.setBrush(QBrush(ToQColor(kUIBackground)));
  painter.drawRoundedRect(rect, 8.0, 8.0);

  // Setup font
  auto font = QFont{"Segoe UI", 9};
  painter.setFont(font);

  const auto x_offset = 35;
  auto y_offset = height() - 245;
  const auto line_height = 20;

  // Header
  font.setBold(true);
  painter.setFont(font);
  painter.setPen(ToQColor(kTextAmber));  // Amber title color
  painter.drawText(x_offset, y_offset, "CONTROLS & SHORTCUTS");
  y_offset += 22;

  font.setBold(false);
  painter.setFont(font);

  struct ShortcutItem {
    QString key;
    QString desc;
  };
  const std::vector<ShortcutItem> items = {{"L-Click + Drag", "Pan Map"},  {"R-Click + Drag", "Rotate Map"},
                                           {"Scroll Wheel", "Zoom Map"},   {"Ctrl+R", "Reset View / Auto-fit"},
                                           {"R", "Toggle Reference Line"}, {"J", "Toggle Junction Boundaries"},
                                           {"B", "Toggle Border Lanes"},   {"O", "Toggle Objects"},
                                           {"S", "Toggle Signals"},        {"L", "Toggle Lanes"}};

  for (const auto& item : items) {
    // Shortcut key
    painter.setPen(ToQColor(kTextValue));  // Light blue/cyan for keys
    painter.drawText(x_offset, y_offset, item.key);

    // Description
    painter.setPen(ToQColor(kTextDescription));  // Slate white for description
    painter.drawText(x_offset + 95, y_offset, item.desc);

    y_offset += line_height;
  }
}

void ViewportWidget::DrawOverlays() {
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing);

  DrawLaneInspector(painter);
  DrawCompass(painter);
  DrawScaleBar(painter);
  DrawShortcutsPanel(painter);
  DrawWaypoints(painter);
  DrawRoutePlannerHUD(painter);
}

void ViewportWidget::DrawWaypoints(QPainter& painter) {
  if (!route_creation_mode_ || waypoint_world_coords_.empty()) {
    return;
  }

  painter.save();
  auto font = QFont{"Segoe UI", 9, QFont::Bold};
  painter.setFont(font);

  int index = 1;
  for (const auto& world_pos : waypoint_world_coords_) {
    const auto screen_pos = camera_.WorldToScreen(static_cast<float>(world_pos.x()), static_cast<float>(world_pos.y()));

    const double radius = 10.0;
    const auto rect = QRectF{screen_pos.x() - radius, screen_pos.y() - radius, radius * 2.0, radius * 2.0};

    painter.setPen(QPen(QColor(245, 197, 61), 2));      // Gold border
    painter.setBrush(QBrush(QColor(15, 23, 42, 220)));  // Slate background
    painter.drawEllipse(rect);

    painter.setPen(Qt::white);
    painter.drawText(rect, Qt::AlignCenter, QString::number(index));

    index++;
  }
  painter.restore();
}

auto ViewportWidget::IsDrivableLane(cpm::RoadId road_id, cpm::LaneId lane_id) const -> bool {
  for (const auto& range : geometry_.mesh_ranges) {
    if (range.road_id == road_id && range.lane_id == lane_id) {
      return range.lane_type == ast::LaneType::kDriving || range.lane_type == ast::LaneType::kOnRamp ||
             range.lane_type == ast::LaneType::kExit || range.lane_type == ast::LaneType::kEntry ||
             range.lane_type == ast::LaneType::kConnectingRamp;
    }
  }
  return false;
}

void ViewportWidget::RecomputeRoute() {
  route_error_.clear();
  if (!routing_graph_.has_value() || waypoint_road_ids_.size() < 2) {
    active_route_ = std::nullopt;
    return;
  }

  auto merged_route = routing::Route{};
  for (std::size_t i = 0; i < waypoint_road_ids_.size() - 1; ++i) {
    const auto path_opt = routing_graph_->FindRoute(waypoint_road_ids_[i], waypoint_road_ids_[i + 1]);
    if (!path_opt.has_value()) {
      active_route_ = std::nullopt;
      route_error_ = "No path found between road " + waypoint_road_ids_[i] + " and " + waypoint_road_ids_[i + 1];
      return;
    }

    for (const auto& seg : path_opt->segments) {
      if (merged_route.segments.empty() || merged_route.segments.back().road_id != seg.road_id) {
        merged_route.segments.push_back(seg);
      }
    }
  }
  active_route_ = merged_route;
}

void ViewportWidget::DrawRoutePlannerHUD(QPainter& painter) {
  if (!route_creation_mode_) {
    return;
  }

  // Determine card height dynamically based on number of waypoints and errors
  const auto num_waypoints = static_cast<int>(waypoint_road_ids_.size());
  constexpr auto base_height = 110;
  const auto waypoint_height = num_waypoints * 20;
  const auto error_height = route_error_.empty() ? 0 : 35;
  const auto card_height = base_height + waypoint_height + error_height;

  constexpr auto card_width = 280;
  const auto card_x = width() - card_width - 20;
  constexpr auto card_y = 100;
  const auto rect = QRect{card_x, card_y, card_width, card_height};

  // Draw dark glassmorphic card container
  painter.save();
  painter.setPen(QPen(ToQColor(kUIBorder), 1));
  painter.setBrush(QBrush(ToQColor(kUIBackground)));
  painter.drawRoundedRect(rect, 8.0, 8.0);

  auto font = QFont{"Segoe UI", 10};
  painter.setFont(font);

  const auto x_offset = card_x + 15;
  auto y_offset = card_y + 25;
  constexpr auto line_height = 20;

  // Header
  font.setBold(true);
  painter.setFont(font);
  painter.setPen(ToQColor(kTextGold));
  painter.drawText(x_offset, y_offset, "ROUTE PLANNER");
  y_offset += 22;

  font.setBold(false);
  painter.setFont(font);

  // Total Route Length
  painter.setPen(ToQColor(kTextLabel));
  painter.drawText(x_offset, y_offset, "Total Length:");
  painter.setPen(ToQColor(kTextValue));
  auto total_length = 0.0;
  if (active_route_.has_value()) {
    for (const auto& seg : active_route_->segments) {
      total_length += seg.length;
    }
  }
  painter.drawText(x_offset + 95, y_offset, QString{"%1 m"}.arg(total_length, 0, 'f', 2));
  y_offset += line_height;

  // Waypoints header
  painter.setPen(ToQColor(kTextLabel));
  painter.drawText(x_offset, y_offset, "Waypoints:");
  y_offset += line_height;

  // List waypoints
  painter.setPen(Qt::white);
  if (waypoint_road_ids_.empty()) {
    painter.setPen(ToQColor(kTextDescription));
    painter.drawText(x_offset + 10, y_offset, "(None)");
    y_offset += line_height;
  } else {
    auto idx = 1;
    for (const auto& road_id : waypoint_road_ids_) {
      painter.drawText(x_offset + 10, y_offset,
                       QString{"Waypoint %1: Road %2"}.arg(idx).arg(QString::fromStdString(road_id)));
      y_offset += line_height;
      idx++;
    }
  }

  // Render pathing errors
  if (!route_error_.empty()) {
    painter.setPen(QPen(QColor(239, 68, 68), 1));       // Red border
    painter.setBrush(QBrush(QColor(239, 68, 68, 30)));  // Red semi-transparent fill
    const auto err_rect = QRect{x_offset, y_offset, card_width - 30, 28};
    painter.drawRoundedRect(err_rect, 4.0, 4.0);

    painter.setPen(QColor(248, 113, 113));  // Light red text
    auto error_font = QFont{"Segoe UI", 9};
    painter.setFont(error_font);
    painter.drawText(err_rect, Qt::AlignCenter, QString::fromStdString(route_error_));

    y_offset += 35;
    painter.setFont(font);  // Restore font
  }

  // Footer with keyboard shortcut reminders
  y_offset = card_y + card_height - 30;
  auto footer_font = QFont{"Segoe UI", 8};
  painter.setFont(footer_font);
  painter.setPen(ToQColor(kTextDescription));
  painter.drawText(x_offset, y_offset, "[Backspace] Undo  [C] Clear  [Esc] Exit");

  painter.restore();
}

}  // namespace strada::vis
