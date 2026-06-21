// SPDX-License-Identifier: BSL-1.0

#include <gtest/gtest.h>

#include <strada/vis/camera.hpp>

namespace strada::vis {

TEST(CameraTest, CameraResetState) {
  Camera camera;
  camera.camera_x = 100.0f;
  camera.camera_y = 200.0f;
  camera.zoom = 5.0f;
  camera.rotation = 45.0f;

  camera.Reset();

  EXPECT_FLOAT_EQ(camera.camera_x, 0.0f);
  EXPECT_FLOAT_EQ(camera.camera_y, 0.0f);
  EXPECT_FLOAT_EQ(camera.zoom, 1.0f);
  EXPECT_FLOAT_EQ(camera.rotation, 0.0f);
}

TEST(CameraTest, ScreenToWorldAndWorldToScreenRoundTrip) {
  Camera camera;
  camera.SetViewport(800, 600);
  camera.camera_x = 50.0f;
  camera.camera_y = -30.0f;
  camera.zoom = 2.5f;
  camera.rotation = 30.0f;

  // Pick a world coordinate
  float world_x = 12.0f;
  float world_y = -45.0f;

  // Convert to screen and back
  auto screen = camera.WorldToScreen(world_x, world_y);
  auto world_back = camera.ScreenToWorld(static_cast<float>(screen.x()), static_cast<float>(screen.y()));

  EXPECT_NEAR(world_back.x(), world_x, 1e-3);
  EXPECT_NEAR(world_back.y(), world_y, 1e-3);
}

TEST(CameraTest, ZoomCenteringInvariance) {
  Camera camera;
  camera.SetViewport(1024, 768);
  camera.camera_x = 10.0f;
  camera.camera_y = 10.0f;
  camera.zoom = 2.0f;
  camera.rotation = 15.0f;

  // Mouse cursor at screen (500, 400)
  float px = 500.0f;
  float py = 400.0f;

  // World point under cursor before zoom
  auto w0 = camera.ScreenToWorld(px, py);

  // Zoom in centered at cursor
  camera.ZoomAt(px, py, 1.2f);  // zoom * 1.2

  // World point under cursor after zoom
  auto w1 = camera.ScreenToWorld(px, py);

  // The world coordinate under the cursor must remain invariant
  EXPECT_NEAR(w0.x(), w1.x(), 1e-3);
  EXPECT_NEAR(w0.y(), w1.y(), 1e-3);
}

TEST(CameraTest, CameraPanning) {
  Camera camera;
  camera.SetViewport(800, 600);
  camera.camera_x = 0.0f;
  camera.camera_y = 0.0f;
  camera.zoom = 2.0f;
  camera.rotation = 90.0f;  // 90 degrees CCW

  // Drag mouse by 10 pixels right, 0 pixels down
  // Since camera is rotated 90 degrees, dragging right shifts the camera in +Y direction in world
  camera.Pan(10.0f, 0.0f);

  // dx_gl = 10, dy_gl = 0. theta = 90 deg.
  // D_w_x = (10 * 0 + 0 * 1) / 2 = 0.
  // D_w_y = (-10 * 1 + 0 * 0) / 2 = -5.
  // C_w_new = C_w_old - D_w = (0, 0) - (0, -5) = (0, 5).
  EXPECT_NEAR(camera.camera_x, 0.0f, 1e-3f);
  EXPECT_NEAR(camera.camera_y, 5.0f, 1e-3f);
}

TEST(CameraTest, CameraRotation) {
  Camera camera;
  camera.rotation = 45.0f;
  camera.Rotate(15.0f);
  EXPECT_FLOAT_EQ(camera.rotation, 60.0f);
}

TEST(CameraTest, ScaleSnapping) {
  // Test various zoom levels
  const double zoom_levels[] = {0.001, 0.005, 0.01, 0.05, 0.1,   0.5,   1.0,   1.55,
                                2.0,   5.0,   10.0, 50.0, 100.0, 500.0, 1000.0};

  for (double zoom : zoom_levels) {
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
  camera.camera_x = 10.0f;
  camera.camera_y = 20.0f;
  camera.zoom = 50.0f;
  camera.rotation = 30.0f;

  QMatrix4x4 view = camera.GetViewMatrix();

  // A point with non-zero Z elevation in world space
  QVector3D p_world(0.0f, 0.0f, 5.0f);
  QVector3D p_view = view.map(p_world);

  // The Z-coordinate should not be scaled by zoom and should remain 5.0f
  EXPECT_FLOAT_EQ(p_view.z(), 5.0f);
}

}  // namespace strada::vis
