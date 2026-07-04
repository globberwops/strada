// SPDX-License-Identifier: BSL-1.0

#include <gtest/gtest.h>

#include <array>
#include <strada/vis/camera.hpp>

namespace strada::vis {

TEST(CameraTest, CameraResetState) {
  Camera camera;
  camera.camera_x = 100.0F;
  camera.camera_y = 200.0F;
  camera.zoom = 5.0F;
  camera.rotation = 45.0F;

  camera.Reset();

  EXPECT_FLOAT_EQ(camera.camera_x, 0.0F);
  EXPECT_FLOAT_EQ(camera.camera_y, 0.0F);
  EXPECT_FLOAT_EQ(camera.zoom, 1.0F);
  EXPECT_FLOAT_EQ(camera.rotation, 0.0F);
}

TEST(CameraTest, ScreenToWorldAndWorldToScreenRoundTrip) {
  Camera camera;
  camera.SetViewport(800, 600);
  camera.camera_x = 50.0F;
  camera.camera_y = -30.0F;
  camera.zoom = 2.5F;
  camera.rotation = 30.0F;

  // Pick a world coordinate
  const float kWorldX = 12.0F;
  const float kWorldY = -45.0F;

  // Convert to screen and back
  const auto kScreen = camera.WorldToScreen(kWorldX, kWorldY);
  const auto kWorldBack = camera.ScreenToWorld(static_cast<float>(kScreen.x()), static_cast<float>(kScreen.y()));

  EXPECT_NEAR(kWorldBack.x(), kWorldX, 1e-3);
  EXPECT_NEAR(kWorldBack.y(), kWorldY, 1e-3);
}

TEST(CameraTest, ZoomCenteringInvariance) {
  Camera camera;
  camera.SetViewport(1024, 768);
  camera.camera_x = 10.0F;
  camera.camera_y = 10.0F;
  camera.zoom = 2.0F;
  camera.rotation = 15.0F;

  // Mouse cursor at screen (500, 400)
  const float kPx = 500.0F;
  const float kPy = 400.0F;

  // World point under cursor before zoom
  const auto kW0 = camera.ScreenToWorld(kPx, kPy);

  // Zoom in centered at cursor
  camera.ZoomAt(kPx, kPy, 1.2F);  // zoom * 1.2

  // World point under cursor after zoom
  const auto kW1 = camera.ScreenToWorld(kPx, kPy);

  // The world coordinate under the cursor must remain invariant
  EXPECT_NEAR(kW0.x(), kW1.x(), 1e-3);
  EXPECT_NEAR(kW0.y(), kW1.y(), 1e-3);
}

TEST(CameraTest, CameraPanning) {
  Camera camera;
  camera.SetViewport(800, 600);
  camera.camera_x = 0.0F;
  camera.camera_y = 0.0F;
  camera.zoom = 2.0F;
  camera.rotation = 90.0F;  // 90 degrees CCW

  // Drag mouse by 10 pixels right, 0 pixels down
  // Since camera is rotated 90 degrees, dragging right shifts the camera in +Y direction in world
  camera.Pan(10.0F, 0.0F);

  // dx_gl = 10, dy_gl = 0. theta = 90 deg.
  // D_w_x = (10 * 0 + 0 * 1) / 2 = 0.
  // D_w_y = (-10 * 1 + 0 * 0) / 2 = -5.
  // C_w_new = C_w_old - D_w = (0, 0) - (0, -5) = (0, 5).
  EXPECT_NEAR(camera.camera_x, 0.0F, 1e-3F);
  EXPECT_NEAR(camera.camera_y, 5.0F, 1e-3F);
}

TEST(CameraTest, CameraRotation) {
  Camera camera;
  camera.rotation = 45.0F;
  camera.Rotate(15.0F);
  EXPECT_FLOAT_EQ(camera.rotation, 60.0F);
}

TEST(CameraTest, ScaleSnapping) {
  // Test various zoom levels
  constexpr std::array<double, 15> kZoomLevels = {0.001, 0.005, 0.01, 0.05, 0.1,   0.5,   1.0,   1.55,
                                                  2.0,   5.0,   10.0, 50.0, 100.0, 500.0, 1000.0};

  for (const double kZoom : kZoomLevels) {
    const double kScaleLength = CalculateScaleLength(kZoom);
    const double kWidthPixels = kScaleLength * kZoom;

    // Check that it's strictly within the 80 to 150 pixel limit
    EXPECT_GE(kWidthPixels, 80.0) << "Failed for zoom: " << kZoom << " (width: " << kWidthPixels << ")";
    EXPECT_LE(kWidthPixels, 150.0) << "Failed for zoom: " << kZoom << " (width: " << kWidthPixels << ")";

    // Verify scale length is positive
    EXPECT_GT(kScaleLength, 0.0);
  }
}

TEST(CameraTest, ViewMatrixZScaleInvariant) {
  Camera camera;
  camera.camera_x = 10.0F;
  camera.camera_y = 20.0F;
  camera.zoom = 50.0F;
  camera.rotation = 30.0F;

  const QMatrix4x4 kView = camera.GetViewMatrix();

  // A point with non-zero Z elevation in world space
  const QVector3D kPWorld(0.0F, 0.0F, 5.0F);
  const QVector3D kPView = kView.map(kPWorld);

  // The Z-coordinate should not be scaled by zoom and should remain 5.0f
  EXPECT_FLOAT_EQ(kPView.z(), 5.0F);
}

}  // namespace strada::vis
