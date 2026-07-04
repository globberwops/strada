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
  float world_x = 12.0F;
  float world_y = -45.0F;

  // Convert to screen and back
  auto screen = camera.WorldToScreen(world_x, world_y);
  auto world_back = camera.ScreenToWorld(static_cast<float>(screen.x()), static_cast<float>(screen.y()));

  EXPECT_NEAR(world_back.x(), world_x, 1e-3);
  EXPECT_NEAR(world_back.y(), world_y, 1e-3);
}

TEST(CameraTest, ZoomCenteringInvariance) {
  Camera camera;
  camera.SetViewport(1024, 768);
  camera.camera_x = 10.0F;
  camera.camera_y = 10.0F;
  camera.zoom = 2.0F;
  camera.rotation = 15.0F;

  // Mouse cursor at screen (500, 400)
  float px = 500.0F;
  float py = 400.0F;

  // World point under cursor before zoom
  auto w0 = camera.ScreenToWorld(px, py);

  // Zoom in centered at cursor
  camera.ZoomAt(px, py, 1.2F);  // zoom * 1.2

  // World point under cursor after zoom
  auto w1 = camera.ScreenToWorld(px, py);

  // The world coordinate under the cursor must remain invariant
  EXPECT_NEAR(w0.x(), w1.x(), 1e-3);
  EXPECT_NEAR(w0.y(), w1.y(), 1e-3);
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

  for (double zoom : kZoomLevels) {
    double scale_length = CalculateScaleLength(zoom);
    double width_pixels = scale_length * zoom;

    // Check that it's strictly within the 80 to 150 pixel limit
    EXPECT_GE(width_pixels, 80.0) << "Failed for zoom: " << zoom << " (width: " << width_pixels << ")";
    EXPECT_LE(width_pixels, 150.0) << "Failed for zoom: " << zoom << " (width: " << width_pixels << ")";

    // Verify scale length is positive
    EXPECT_GT(scale_length, 0.0);
  }
}

TEST(CameraTest, ViewMatrixZScaleInvariant) {
  Camera camera;
  camera.camera_x = 10.0F;
  camera.camera_y = 20.0F;
  camera.zoom = 50.0F;
  camera.rotation = 30.0F;

  QMatrix4x4 view = camera.GetViewMatrix();

  // A point with non-zero Z elevation in world space
  QVector3D p_world(0.0F, 0.0F, 5.0F);
  QVector3D p_view = view.map(p_world);

  // The Z-coordinate should not be scaled by zoom and should remain 5.0f
  EXPECT_FLOAT_EQ(p_view.z(), 5.0F);
}

}  // namespace strada::vis
