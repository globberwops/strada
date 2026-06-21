// SPDX-License-Identifier: BSL-1.0

#include <strada/vis/camera.hpp>

namespace strada::vis {

void Camera::Reset() noexcept {
  camera_x = 0.0f;
  camera_y = 0.0f;
  zoom = 1.0f;
  rotation = 0.0f;
}

void Camera::SetViewport(int width, int height) noexcept {
  viewport_width = width;
  viewport_height = height;
}

void Camera::ZoomAt(float screen_x, float screen_y, float factor) noexcept {
  QPointF w = ScreenToWorld(screen_x, screen_y);
  zoom *= factor;
  zoom = std::max(0.001f, std::min(zoom, 1000.0f));

  float dx = screen_x - (static_cast<float>(viewport_width) / 2.0f);
  float dy = (static_cast<float>(viewport_height) / 2.0f) - screen_y;

  float rad = rotation * static_cast<float>(M_PI) / 180.0f;
  float cos_val = std::cos(rad);
  float sin_val = std::sin(rad);

  float rx = (dx * cos_val) + (dy * sin_val);
  float ry = (-dx * sin_val) + (dy * cos_val);

  camera_x = static_cast<float>(w.x()) - (rx / zoom);
  camera_y = static_cast<float>(w.y()) - (ry / zoom);
}

void Camera::Pan(float delta_screen_x, float delta_screen_y) noexcept {
  float dx_gl = delta_screen_x;
  float dy_gl = -delta_screen_y;

  float rad = rotation * static_cast<float>(M_PI) / 180.0f;
  float cos_val = std::cos(rad);
  float sin_val = std::sin(rad);

  float dw_x = ((dx_gl * cos_val) + (dy_gl * sin_val)) / zoom;
  float dw_y = ((-dx_gl * sin_val) + (dy_gl * cos_val)) / zoom;

  camera_x -= dw_x;
  camera_y -= dw_y;
}

void Camera::Rotate(float delta_degrees) noexcept {
  rotation += delta_degrees;
  while (rotation >= 360.0f) {
    rotation -= 360.0f;
  }
  while (rotation < 0.0f) {
    rotation += 360.0f;
  }
}

auto Camera::ScreenToWorld(float screen_x, float screen_y) const noexcept -> QPointF {
  float dx = screen_x - (static_cast<float>(viewport_width) / 2.0f);
  float dy = (static_cast<float>(viewport_height) / 2.0f) - screen_y;

  float rad = rotation * static_cast<float>(M_PI) / 180.0f;
  float cos_val = std::cos(rad);
  float sin_val = std::sin(rad);

  float rx = (dx * cos_val) + (dy * sin_val);
  float ry = (-dx * sin_val) + (dy * cos_val);

  float wx = rx / zoom;
  float wy = ry / zoom;

  return QPointF{camera_x + wx, camera_y + wy};
}

auto Camera::WorldToScreen(float world_x, float world_y) const noexcept -> QPointF {
  float wx = world_x - camera_x;
  float wy = world_y - camera_y;

  float rad = rotation * static_cast<float>(M_PI) / 180.0f;
  float cos_val = std::cos(rad);
  float sin_val = std::sin(rad);

  float rx = ((wx * cos_val) - (wy * sin_val)) * zoom;
  float ry = ((wx * sin_val) + (wy * cos_val)) * zoom;

  float screen_x = (static_cast<float>(viewport_width) / 2.0f) + rx;
  float screen_y = (static_cast<float>(viewport_height) / 2.0f) - ry;

  return QPointF{screen_x, screen_y};
}

auto Camera::GetProjectionMatrix() const noexcept -> QMatrix4x4 {
  QMatrix4x4 projection;
  projection.ortho(-static_cast<float>(viewport_width) / 2.0f, static_cast<float>(viewport_width) / 2.0f,
                   -static_cast<float>(viewport_height) / 2.0f, static_cast<float>(viewport_height) / 2.0f, -100.0f,
                   100.0f);
  return projection;
}

auto Camera::GetViewMatrix() const noexcept -> QMatrix4x4 {
  QMatrix4x4 view;
  view.scale(zoom);
  view.rotate(rotation, 0.0f, 0.0f, 1.0f);
  view.translate(-camera_x, -camera_y, 0.0f);
  return view;
}

}  // namespace strada::vis
