// SPDX-License-Identifier: BSL-1.0

#pragma once

#include <QMatrix4x4>
#include <QPointF>

namespace strada::vis {

/// Encapsulates the 2D orthographic camera properties and screen-world coordinate projections.
class Camera {
 public:
  /// Default constructor.
  Camera() = default;

  /// Resets the camera to default values.
  void Reset() noexcept;

  /// Sets the viewport dimensions.
  void SetViewport(int width, int height) noexcept;

  /// Updates zoom factor centered around a screen coordinate (mouse cursor position).
  void ZoomAt(float screen_x, float screen_y, float factor) noexcept;

  /// Translates camera target in screen-space delta.
  void Pan(float delta_screen_x, float delta_screen_y) noexcept;

  /// Updates camera view rotation angle (in degrees).
  void Rotate(float delta_degrees) noexcept;

  /// Converts screen pixel coordinates (Qt: origin top-left, Y down) to world Inertial coordinates (X, Y).
  [[nodiscard]] auto ScreenToWorld(float screen_x, float screen_y) const noexcept -> QPointF;

  /// Converts world Inertial coordinates (X, Y) to screen pixel coordinates.
  [[nodiscard]] auto WorldToScreen(float world_x, float world_y) const noexcept -> QPointF;

  /// Returns the orthographic projection matrix.
  [[nodiscard]] auto GetProjectionMatrix() const noexcept -> QMatrix4x4;

  /// Returns the view translation, zoom, and rotation matrix.
  [[nodiscard]] auto GetViewMatrix() const noexcept -> QMatrix4x4;

  // View parameters
  float camera_x{0.0F};    ///< Target center in world X.
  float camera_y{0.0F};    ///< Target center in world Y.
  float zoom{1.0F};        ///< Zoom scale factor (pixels per world unit).
  float rotation{0.0F};    ///< Rotation angle in degrees (clockwise).
  int viewport_width{1};   ///< Viewport width in pixels.
  int viewport_height{1};  ///< Viewport height in pixels.
};

/// Computes a nice rounded physical scale length (in meters) for a given zoom level
/// such that its screen width (scale_length * zoom) is within [80, 150] pixels.
[[nodiscard]] auto CalculateScaleLength(double zoom) noexcept -> double;

}  // namespace strada::vis
