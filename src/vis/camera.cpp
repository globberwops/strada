// SPDX-License-Identifier: BSL-1.0

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numbers>
#include <strada/vis/camera.hpp>

namespace strada::vis {

namespace {
constexpr float kProjectionLimit = 10000000.0F;
constexpr double kIdealScaleWidth = 115.0;
constexpr double kMinScaleWidth = 80.0;
constexpr double kMaxScaleWidth = 150.0;
constexpr double kBase10 = 10.0;
}  // namespace

void Camera::Reset() noexcept {
  camera_x = 0.0F;
  camera_y = 0.0F;
  zoom = 1.0F;
  rotation = 0.0F;
}

void Camera::SetViewport(int width, int height) noexcept {
  viewport_width = width;
  viewport_height = height;
}

void Camera::ZoomAt(float screen_x, float screen_y, float factor) noexcept {
  QPointF const w = ScreenToWorld(screen_x, screen_y);
  zoom *= factor;
  zoom = std::max(0.001F, std::min(zoom, 1000.0F));

  float const dx = screen_x - (static_cast<float>(viewport_width) / 2.0F);
  float const dy = (static_cast<float>(viewport_height) / 2.0F) - screen_y;

  float const rad = rotation * std::numbers::pi_v<float> / 180.0F;
  float const cos_val = std::cos(rad);
  float const sin_val = std::sin(rad);

  float const rx = (dx * cos_val) + (dy * sin_val);
  float const ry = (-dx * sin_val) + (dy * cos_val);

  camera_x = static_cast<float>(w.x()) - (rx / zoom);
  camera_y = static_cast<float>(w.y()) - (ry / zoom);
}

void Camera::Pan(float delta_screen_x, float delta_screen_y) noexcept {
  float const dx_gl = delta_screen_x;
  float const dy_gl = -delta_screen_y;

  float const rad = rotation * std::numbers::pi_v<float> / 180.0F;
  float const cos_val = std::cos(rad);
  float const sin_val = std::sin(rad);

  float const dw_x = ((dx_gl * cos_val) + (dy_gl * sin_val)) / zoom;
  float const dw_y = ((-dx_gl * sin_val) + (dy_gl * cos_val)) / zoom;

  camera_x -= dw_x;
  camera_y -= dw_y;
}

void Camera::Rotate(float delta_degrees) noexcept {
  rotation += delta_degrees;
  while (rotation >= 360.0F) {
    rotation -= 360.0F;
  }
  while (rotation < 0.0F) {
    rotation += 360.0F;
  }
}

auto Camera::ScreenToWorld(float screen_x, float screen_y) const noexcept -> QPointF {
  float const dx = screen_x - (static_cast<float>(viewport_width) / 2.0F);
  float const dy = (static_cast<float>(viewport_height) / 2.0F) - screen_y;

  float const rad = rotation * std::numbers::pi_v<float> / 180.0F;
  float const cos_val = std::cos(rad);
  float const sin_val = std::sin(rad);

  float const rx = (dx * cos_val) + (dy * sin_val);
  float const ry = (-dx * sin_val) + (dy * cos_val);

  float const wx = rx / zoom;
  float const wy = ry / zoom;

  return QPointF{camera_x + wx, camera_y + wy};
}

auto Camera::WorldToScreen(float world_x, float world_y) const noexcept -> QPointF {
  float const wx = world_x - camera_x;
  float const wy = world_y - camera_y;

  float const rad = rotation * std::numbers::pi_v<float> / 180.0F;
  float const cos_val = std::cos(rad);
  float const sin_val = std::sin(rad);

  float const rx = ((wx * cos_val) - (wy * sin_val)) * zoom;
  float const ry = ((wx * sin_val) + (wy * cos_val)) * zoom;

  float const screen_x = (static_cast<float>(viewport_width) / 2.0F) + rx;
  float const screen_y = (static_cast<float>(viewport_height) / 2.0F) - ry;

  return QPointF{screen_x, screen_y};
}

auto Camera::GetProjectionMatrix() const noexcept -> QMatrix4x4 {
  QMatrix4x4 projection;
  projection.ortho(-static_cast<float>(viewport_width) / 2.0F, static_cast<float>(viewport_width) / 2.0F,
                   -static_cast<float>(viewport_height) / 2.0F, static_cast<float>(viewport_height) / 2.0F,
                   -kProjectionLimit, kProjectionLimit);
  return projection;
}

auto Camera::GetViewMatrix() const noexcept -> QMatrix4x4 {
  QMatrix4x4 view;
  view.scale(zoom, zoom, 1.0F);
  view.rotate(rotation, 0.0F, 0.0F, 1.0F);
  view.translate(-camera_x, -camera_y, 0.0F);
  return view;
}

auto CalculateScaleLength(double zoom) noexcept -> double {
  if (zoom <= 0.0) {
    return 1.0;
  }
  // Candidates in one decade
  const std::array<double, 6> kFactors = {1.0, 1.5, 2.0, 3.0, 5.0, 7.5};

  // Find a starting decade
  double const l_ideal = kIdealScaleWidth / zoom;
  double const exponent = std::floor(std::log10(l_ideal));
  double const base = std::pow(kBase10, exponent);

  // Search across three decades around the ideal to find the one
  // that results in a screen width strictly within [80, 150].
  // If multiple do, pick the one closest to 115 pixels.
  double best_val = 0.0;
  double best_dist_to_ideal = std::numeric_limits<double>::max();
  bool found_in_range = false;

  for (int decade = -1; decade <= 1; ++decade) {
    double const scale_mult = base * std::pow(kBase10, decade);
    for (double const f : kFactors) {
      double const candidate = f * scale_mult;
      double const width = candidate * zoom;
      if (width >= kMinScaleWidth && width <= kMaxScaleWidth) {
        found_in_range = true;
        double const dist = std::abs(width - kIdealScaleWidth);
        if (dist < best_dist_to_ideal) {
          best_dist_to_ideal = dist;
          best_val = candidate;
        }
      }
    }
  }

  // Fallback if none is strictly in range
  // (mathematically shouldn't happen with these factors)
  if (!found_in_range) {
    double min_diff = std::numeric_limits<double>::max();
    for (int decade = -2; decade <= 2; ++decade) {
      double const scale_mult = base * std::pow(kBase10, decade);
      for (double const f : kFactors) {
        double const candidate = f * scale_mult;
        double const width = candidate * zoom;
        double const diff = std::abs(width - kIdealScaleWidth);
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
