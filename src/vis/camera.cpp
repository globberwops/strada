// SPDX-License-Identifier: BSL-1.0

#include <cmath>
#include <limits>
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
  view.scale(zoom, zoom, 1.0f);
  view.rotate(rotation, 0.0f, 0.0f, 1.0f);
  view.translate(-camera_x, -camera_y, 0.0f);
  return view;
}

auto CalculateScaleLength(double zoom) noexcept -> double {
  if (zoom <= 0.0) {
    return 1.0;
  }
  // Candidates in one decade
  const double factors[] = {1.0, 1.5, 2.0, 3.0, 5.0, 7.5};

  // Find a starting decade
  double l_ideal = 115.0 / zoom;
  double exponent = std::floor(std::log10(l_ideal));
  double base = std::pow(10.0, exponent);

  // Search across three decades around the ideal to find the one
  // that results in a screen width strictly within [80, 150].
  // If multiple do, pick the one closest to 115 pixels.
  double best_val = 0.0;
  double best_dist_to_ideal = std::numeric_limits<double>::max();
  bool found_in_range = false;

  for (int decade = -1; decade <= 1; ++decade) {
    double scale_mult = base * std::pow(10.0, decade);
    for (double f : factors) {
      double candidate = f * scale_mult;
      double width = candidate * zoom;
      if (width >= 80.0 && width <= 150.0) {
        found_in_range = true;
        double dist = std::abs(width - 115.0);
        if (dist < best_dist_to_ideal) {
          best_dist_to_ideal = dist;
          best_val = candidate;
        }
      }
    }
  }

  // Fallback if none is strictly in range (mathematically shouldn't happen with these factors)
  if (!found_in_range) {
    double min_diff = std::numeric_limits<double>::max();
    for (int decade = -2; decade <= 2; ++decade) {
      double scale_mult = base * std::pow(10.0, decade);
      for (double f : factors) {
        double candidate = f * scale_mult;
        double width = candidate * zoom;
        double diff = std::abs(width - 115.0);
        if (diff < min_diff) {
          min_diff = diff;
          best_val = candidate;
        }
      }
    }
  }

  return best_val;
}

}  // namespace strada::vis
