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
  const QPointF kW = ScreenToWorld(screen_x, screen_y);
  zoom *= factor;
  zoom = std::max(0.001F, std::min(zoom, 1000.0F));

  const float kDx = screen_x - (static_cast<float>(viewport_width) / 2.0F);
  const float kDy = (static_cast<float>(viewport_height) / 2.0F) - screen_y;

  const float kRad = rotation * std::numbers::pi_v<float> / 180.0F;
  const float kCosVal = std::cos(kRad);
  const float kSinVal = std::sin(kRad);

  const float kRx = (kDx * kCosVal) + (kDy * kSinVal);
  const float kRy = (-kDx * kSinVal) + (kDy * kCosVal);

  camera_x = static_cast<float>(kW.x()) - (kRx / zoom);
  camera_y = static_cast<float>(kW.y()) - (kRy / zoom);
}

void Camera::Pan(float delta_screen_x, float delta_screen_y) noexcept {
  const float kDxGl = delta_screen_x;
  const float kDyGl = -delta_screen_y;

  const float kRad = rotation * std::numbers::pi_v<float> / 180.0F;
  const float kCosVal = std::cos(kRad);
  const float kSinVal = std::sin(kRad);

  const float kDwX = ((kDxGl * kCosVal) + (kDyGl * kSinVal)) / zoom;
  const float kDwY = ((-kDxGl * kSinVal) + (kDyGl * kCosVal)) / zoom;

  camera_x -= kDwX;
  camera_y -= kDwY;
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
  const float kDx = screen_x - (static_cast<float>(viewport_width) / 2.0F);
  const float kDy = (static_cast<float>(viewport_height) / 2.0F) - screen_y;

  const float kRad = rotation * std::numbers::pi_v<float> / 180.0F;
  const float kCosVal = std::cos(kRad);
  const float kSinVal = std::sin(kRad);

  const float kRx = (kDx * kCosVal) + (kDy * kSinVal);
  const float kRy = (-kDx * kSinVal) + (kDy * kCosVal);

  const float kWx = kRx / zoom;
  const float kWy = kRy / zoom;

  return QPointF{camera_x + kWx, camera_y + kWy};
}

auto Camera::WorldToScreen(float world_x, float world_y) const noexcept -> QPointF {
  const float kWx = world_x - camera_x;
  const float kWy = world_y - camera_y;

  const float kRad = rotation * std::numbers::pi_v<float> / 180.0F;
  const float kCosVal = std::cos(kRad);
  const float kSinVal = std::sin(kRad);

  const float kRx = ((kWx * kCosVal) - (kWy * kSinVal)) * zoom;
  const float kRy = ((kWx * kSinVal) + (kWy * kCosVal)) * zoom;

  const float kScreenX = (static_cast<float>(viewport_width) / 2.0F) + kRx;
  const float kScreenY = (static_cast<float>(viewport_height) / 2.0F) - kRy;

  return QPointF{kScreenX, kScreenY};
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
  const double kLIdeal = kIdealScaleWidth / zoom;
  const double kExponent = std::floor(std::log10(kLIdeal));
  const double kBase = std::pow(kBase10, kExponent);

  // Search across three decades around the ideal to find the one
  // that results in a screen width strictly within [80, 150].
  // If multiple do, pick the one closest to 115 pixels.
  double best_val = 0.0;
  double best_dist_to_ideal = std::numeric_limits<double>::max();
  bool found_in_range = false;

  for (int decade = -1; decade <= 1; ++decade) {
    const double kScaleMult = kBase * std::pow(kBase10, decade);
    for (const double kF : kFactors) {
      const double kCandidate = kF * kScaleMult;
      const double kWidth = kCandidate * zoom;
      if (kWidth >= kMinScaleWidth && kWidth <= kMaxScaleWidth) {
        found_in_range = true;
        const double kDist = std::abs(kWidth - kIdealScaleWidth);
        if (kDist < best_dist_to_ideal) {
          best_dist_to_ideal = kDist;
          best_val = kCandidate;
        }
      }
    }
  }

  // Fallback if none is strictly in range
  // (mathematically shouldn't happen with these factors)
  if (!found_in_range) {
    double min_diff = std::numeric_limits<double>::max();
    for (int decade = -2; decade <= 2; ++decade) {
      const double kScaleMult = kBase * std::pow(kBase10, decade);
      for (const double kF : kFactors) {
        const double kCandidate = kF * kScaleMult;
        const double kWidth = kCandidate * zoom;
        const double kDiff = std::abs(kWidth - kIdealScaleWidth);
        if (kDiff < min_diff) {
          min_diff = kDiff;
          best_val = kCandidate;
        }
      }
    }
  }

  return best_val;
}

}  // namespace strada::vis
