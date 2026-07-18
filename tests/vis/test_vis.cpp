#include <gtest/gtest.h>

#include <QApplication>
#include <QKeyEvent>
#include <QMouseEvent>
#include <filesystem>
#include <strada/parser/parser.hpp>
#include <strada/vis/geometry_batcher.hpp>
#include <strada/vis/viewport_widget.hpp>

#ifndef STRADA_TEST_DATA_DIR
#define STRADA_TEST_DATA_DIR "tests/data"
#endif

namespace strada::vis {

TEST(VisTest, LaneColorDistinctiveness) {
  // Arrange
  auto driving_pos = GetLaneColor(ast::LaneType::kDriving, -1);
  auto driving_neg = GetLaneColor(ast::LaneType::kDriving, 1);
  auto sidewalk_color = GetLaneColor(ast::LaneType::kSidewalk, 0);
  auto shoulder_color = GetLaneColor(ast::LaneType::kShoulder, 0);
  auto none_color = GetLaneColor(ast::LaneType::kNone, 0);

  // Assert: colors should not be identical
  EXPECT_NE(driving_pos.r, driving_neg.r);
  EXPECT_NE(driving_pos.r, sidewalk_color.r);
  EXPECT_NE(sidewalk_color.r, shoulder_color.r);
  EXPECT_NE(driving_pos.r, none_color.r);
}

TEST(VisTest, LaneColorMapping) {
  // Test driving + (original_lane_id < 0)
  auto driving_pos = GetLaneColor(ast::LaneType::kDriving, -1);
  EXPECT_NEAR(driving_pos.r, 239.0F / 255.0F, 1e-4F);
  EXPECT_NEAR(driving_pos.g, 215.0F / 255.0F, 1e-4F);
  EXPECT_NEAR(driving_pos.b, 171.0F / 255.0F, 1e-4F);

  // Test driving - (original_lane_id > 0)
  auto driving_neg = GetLaneColor(ast::LaneType::kDriving, 1);
  EXPECT_NEAR(driving_neg.r, 205.0F / 255.0F, 1e-4F);
  EXPECT_NEAR(driving_neg.g, 216.0F / 255.0F, 1e-4F);
  EXPECT_NEAR(driving_neg.b, 232.0F / 255.0F, 1e-4F);

  // Test unspec-defined lane types mapping to driving
  auto hov_color = GetLaneColor(ast::LaneType::kHov, -1);
  EXPECT_NEAR(hov_color.r, driving_pos.r, 1e-4F);

  // Test biking mapping
  auto biking_color = GetLaneColor(ast::LaneType::kBiking, 0);
  EXPECT_NEAR(biking_color.r, 207.0F / 255.0F, 1e-4F);
  EXPECT_NEAR(biking_color.g, 16.0F / 255.0F, 1e-4F);
  EXPECT_NEAR(biking_color.b, 45.0F / 255.0F, 1e-4F);

  // Test none mapping
  auto none_color = GetLaneColor(ast::LaneType::kNone, 0);
  EXPECT_NEAR(none_color.r, 147.0F / 255.0F, 1e-4F);
  EXPECT_NEAR(none_color.g, 149.0F / 255.0F, 1e-4F);
  EXPECT_NEAR(none_color.b, 152.0F / 255.0F, 1e-4F);
}

TEST(VisTest, BatchMapGeometryTriangulation) {
  // Arrange: programmatically construct an AST with a road and a left lane
  ast::AbstractSyntaxTree map;

  ast::Road road;
  road.id = "1";
  road.length = 10.0;

  ast::GeometryRecord geom;
  geom.s = 0.0;
  geom.length = 10.0;
  geom.x = 0.0;
  geom.y = 0.0;
  geom.hdg = 0.0;
  geom.shape = ast::Line{};
  road.plan_view.push_back(geom);

  ast::LaneSection section;
  section.s = 0.0;

  ast::Lane lane1;
  lane1.id = 1;
  lane1.type = strada::ast::LaneType::kDriving;
  ast::LaneWidth w1;
  w1.s_offset = 0.0;
  w1.a = 3.0;
  lane1.widths.push_back(w1);
  section.left.push_back(lane1);

  ast::Lane lane0;
  lane0.id = 0;
  lane0.type = strada::ast::LaneType::kBorder;
  section.center.push_back(lane0);

  road.lanes.sections.push_back(section);
  map.roads.push_back(road);

  cpm::CompiledPhysicsModel cpm(map);
  tess::Tessellator tess(map, cpm, 0.5);
  ASSERT_EQ(tess.Meshes().size(), 1);

  // Act: batch geometry
  auto batched = BatchMapGeometry(tess);

  // Assert
  const auto& mesh = tess.Meshes()[0];
  EXPECT_EQ(batched.triangle_vertices.size(), mesh.vertices.size());
  EXPECT_EQ(batched.triangle_indices.size(), mesh.indices.size());

  // Verify colors are mapped (driving is dark grey/blue)
  auto color = GetLaneColor(ast::LaneType::kDriving, 1);
  for (const auto& v : batched.triangle_vertices) {
    EXPECT_NEAR(v.r, color.r, 1e-4F);
    EXPECT_NEAR(v.g, color.g, 1e-4F);
    EXPECT_NEAR(v.b, color.b, 1e-4F);
  }

  // Verify indices are identical since we only have one mesh
  for (std::size_t i = 0; i < mesh.indices.size(); ++i) {
    EXPECT_EQ(batched.triangle_indices[i], mesh.indices[i]);
  }
}

TEST(VisTest, BatchMapGeometryLines) {
  // Arrange: programmatically construct an AST with a road and a left lane
  ast::AbstractSyntaxTree map;

  ast::Road road;
  road.id = "1";
  road.length = 10.0;

  ast::GeometryRecord geom;
  geom.s = 0.0;
  geom.length = 10.0;
  geom.x = 0.0;
  geom.y = 0.0;
  geom.hdg = 0.0;
  geom.shape = ast::Line{};
  road.plan_view.push_back(geom);

  ast::LaneSection section;
  section.s = 0.0;

  ast::Lane lane1;
  lane1.id = 1;
  lane1.type = strada::ast::LaneType::kDriving;
  ast::LaneWidth w1;
  w1.s_offset = 0.0;
  w1.a = 3.0;
  lane1.widths.push_back(w1);
  section.left.push_back(lane1);

  ast::Lane lane0;
  lane0.id = 0;
  lane0.type = strada::ast::LaneType::kBorder;
  section.center.push_back(lane0);

  road.lanes.sections.push_back(section);
  map.roads.push_back(road);

  cpm::CompiledPhysicsModel cpm(map);
  tess::Tessellator tess(map, cpm, 0.5);
  // Expect 1 reference line polyline and 1 outer boundary polyline
  ASSERT_EQ(tess.Polylines().size(), 2);

  // Act: batch
  auto batched = BatchMapGeometry(tess);

  // Assert:
  // For each polyline of M vertices, we generate 2*(M-1) vertices for GL_LINES
  std::size_t expected_line_verts = 0;
  for (const auto& poly : tess.Polylines()) {
    if (poly.is_reference_line) {
      expected_line_verts += 2 * (poly.vertices.size() - 1);
    }
  }
  EXPECT_EQ(batched.line_vertices.size(), expected_line_verts);

  // Check line segment pairings and colors
  // Only the reference line should be present (yellow)
  std::size_t idx = 0;
  for (const auto& poly : tess.Polylines()) {
    if (!poly.is_reference_line) {
      continue;
    }
    for (std::size_t i = 0; i < poly.vertices.size() - 1; ++i) {
      const auto& v_start = batched.line_vertices[idx++];
      const auto& v_end = batched.line_vertices[idx++];

      // Verify coordinate data matches
      EXPECT_NEAR(v_start.x, poly.vertices[i].x, 1e-4F);
      EXPECT_NEAR(v_end.x, poly.vertices[i + 1].x, 1e-4F);

      // Red color
      EXPECT_NEAR(v_start.r, 1.0F, 1e-4F);
      EXPECT_NEAR(v_start.g, 0.0F, 1e-4F);
      EXPECT_NEAR(v_start.b, 0.0F, 1e-4F);
    }
  }
}

TEST(VisTest, MeshRangeTracking) {
  // Arrange: programmatically construct an AST with a road and a left lane
  ast::AbstractSyntaxTree map;

  ast::Road road;
  road.id = "1";
  road.length = 10.0;

  ast::GeometryRecord geom;
  geom.s = 0.0;
  geom.length = 10.0;
  geom.x = 0.0;
  geom.y = 0.0;
  geom.hdg = 0.0;
  geom.shape = ast::Line{};
  road.plan_view.push_back(geom);

  ast::LaneSection section;
  section.s = 0.0;

  ast::Lane lane1;
  lane1.id = 1;
  lane1.type = strada::ast::LaneType::kDriving;
  ast::LaneWidth w1;
  w1.s_offset = 0.0;
  w1.a = 3.0;
  lane1.widths.push_back(w1);
  section.left.push_back(lane1);

  ast::Lane lane0;
  lane0.id = 0;
  lane0.type = strada::ast::LaneType::kBorder;
  section.center.push_back(lane0);

  road.lanes.sections.push_back(section);
  map.roads.push_back(road);

  cpm::CompiledPhysicsModel cpm(map);
  tess::Tessellator tess(map, cpm, 0.5);
  ASSERT_EQ(tess.Meshes().size(), 1);

  // Act: batch geometry
  auto batched = BatchMapGeometry(tess);

  // Assert: we expect exactly 1 range corresponding to the single mesh
  ASSERT_EQ(batched.mesh_ranges.size(), 1);
  const auto& range = batched.mesh_ranges[0];

  EXPECT_EQ(range.road_id, tess.Meshes()[0].road_id);
  EXPECT_EQ(range.lane_id, tess.Meshes()[0].lane_id);
  EXPECT_EQ(range.lane_type, ast::LaneType::kDriving);
  EXPECT_EQ(range.index_start, 0);
  EXPECT_EQ(range.index_count, tess.Meshes()[0].indices.size());
}

TEST(VisTest, BatchMapGeometryJunctionBoundaries) {
  // Arrange
  auto map_path = std::filesystem::path(STRADA_TEST_DATA_DIR) / "junction_boundary.xodr";
  auto map = parser::ParseFile(map_path);

  // Act
  cpm::CompiledPhysicsModel cpm(map);
  tess::Tessellator tess(map, cpm, 0.5);
  auto batched = BatchMapGeometry(tess);

  // Assert
  EXPECT_FALSE(batched.boundary_triangle_vertices.empty());
  EXPECT_FALSE(batched.boundary_triangle_indices.empty());

  // Check color matches amber (245, 197, 61)
  for (const auto& v : batched.boundary_triangle_vertices) {
    EXPECT_NEAR(v.r, 245.0F / 255.0F, 1e-4F);
    EXPECT_NEAR(v.g, 197.0F / 255.0F, 1e-4F);
    EXPECT_NEAR(v.b, 61.0F / 255.0F, 1e-4F);
  }
}

TEST(VisTest, BatchMapGeometryObjects) {
  // Arrange
  ast::AbstractSyntaxTree map;

  ast::Road road;
  road.id = "1";
  road.length = 10.0;

  ast::GeometryRecord geom;
  geom.s = 0.0;
  geom.length = 10.0;
  geom.x = 0.0;
  geom.y = 0.0;
  geom.hdg = 0.0;
  geom.shape = ast::Line{};
  road.plan_view.push_back(geom);

  ast::LaneSection section;
  section.s = 0.0;
  ast::Lane lane0;
  lane0.id = 0;
  lane0.type = strada::ast::LaneType::kBorder;
  section.center.push_back(lane0);
  road.lanes.sections.push_back(section);

  // 1. Add object with local corners (Outline)
  ast::Object obj_local;
  obj_local.id = "obj_local";
  obj_local.s = 2.0;
  obj_local.t = 0.0;
  obj_local.z_offset = 1.0;
  obj_local.hdg = 0.0;
  obj_local.pitch = 0.0;
  obj_local.roll = 0.0;

  ast::ObjectOutline outline_local;
  outline_local.closed = true;
  outline_local.corners_local.push_back(ast::ObjectCornerLocal{.u = -1.0, .v = -1.0, .z = 0.0});
  outline_local.corners_local.push_back(ast::ObjectCornerLocal{.u = 1.0, .v = -1.0, .z = 0.0});
  outline_local.corners_local.push_back(ast::ObjectCornerLocal{.u = 1.0, .v = 1.0, .z = 0.0});
  outline_local.corners_local.push_back(ast::ObjectCornerLocal{.u = -1.0, .v = 1.0, .z = 0.0});
  obj_local.outlines.push_back(outline_local);
  road.objects.push_back(obj_local);

  // 2. Add object with road corners (Outline)
  ast::Object obj_road;
  obj_road.id = "obj_road";
  obj_road.s = 4.0;
  obj_road.t = 0.0;
  obj_road.z_offset = 0.0;

  ast::ObjectOutline outline_road;
  outline_road.closed = false;
  outline_road.corners_road.push_back(ast::ObjectCornerRoad{.s = 4.0, .t = -1.0, .dz = 0.0});
  outline_road.corners_road.push_back(ast::ObjectCornerRoad{.s = 5.0, .t = 1.0, .dz = 0.0});
  obj_road.outlines.push_back(outline_road);
  road.objects.push_back(obj_road);

  // 3. Add object with length/width > 0 (Oriented 2D Box)
  ast::Object obj_box;
  obj_box.id = "obj_box";
  obj_box.s = 6.0;
  obj_box.t = 0.0;
  obj_box.z_offset = 0.0;
  obj_box.length = 2.0;
  obj_box.width = 1.0;
  obj_box.hdg = 0.0;
  obj_box.pitch = 0.0;
  obj_box.roll = 0.0;
  road.objects.push_back(obj_box);

  // 4. Add other object (Crosshair)
  ast::Object obj_cross;
  obj_cross.id = "obj_cross";
  obj_cross.s = 8.0;
  obj_cross.t = 0.0;
  obj_cross.z_offset = 0.0;
  road.objects.push_back(obj_cross);

  map.roads.push_back(road);

  cpm::CompiledPhysicsModel cpm(map);
  tess::Tessellator tess(map, cpm, 0.5);

  // Act
  auto batched = BatchMapGeometry(tess);

  // Assert
  // 1. obj_local has 4 local corners, closed -> 4 line segments -> 8 vertices
  // 2. obj_road has 2 road corners, open -> 1 line segment -> 2 vertices
  // 3. obj_box has length/width > 0 -> 4 line segments -> 8 vertices
  // 4. obj_cross has no outline/dim -> crosshair -> 2 line segments -> 4 vertices
  // Total expected vertices = 8 + 2 + 8 + 4 = 22 vertices.
  EXPECT_EQ(batched.object_line_vertices.size(), 22);

  // Verify colors are neon amber/orange: rgb(255, 145, 0)
  for (const auto& v : batched.object_line_vertices) {
    EXPECT_NEAR(v.r, 1.0F, 1e-4F);
    EXPECT_NEAR(v.g, 145.0F / 255.0F, 1e-4F);
    EXPECT_NEAR(v.b, 0.0F, 1e-4F);
  }
}

TEST(VisTest, FindActiveRoadType) {
  // Arrange
  ast::Road road;
  road.id = "1";
  road.length = 10.0;

  // Case 1: Empty types vector
  EXPECT_EQ(ViewportWidget::FindActiveRoadType(road, 0.0), ast::RoadType::kUnknown);
  EXPECT_EQ(ViewportWidget::FindActiveRoadType(road, 5.0), ast::RoadType::kUnknown);

  // Set up types
  road.types.push_back(ast::RoadTypeRecord{.s = 0.0, .type = ast::RoadType::kTownLocal});
  road.types.push_back(ast::RoadTypeRecord{.s = 2.0, .type = ast::RoadType::kBicycle});
  road.types.push_back(ast::RoadTypeRecord{.s = 5.0, .type = ast::RoadType::kTownExpressway});

  // Case 2: Before first record (s < 0.0) -> returns kUnknown per our logic
  EXPECT_EQ(ViewportWidget::FindActiveRoadType(road, -1.0), ast::RoadType::kUnknown);

  // Case 3: Exact match at s = 0.0 -> kTownLocal
  EXPECT_EQ(ViewportWidget::FindActiveRoadType(road, 0.0), ast::RoadType::kTownLocal);

  // Case 4: Match between 0.0 and 2.0 -> kTownLocal
  EXPECT_EQ(ViewportWidget::FindActiveRoadType(road, 1.0), ast::RoadType::kTownLocal);

  // Case 5: Exact match at s = 2.0 -> kBicycle
  EXPECT_EQ(ViewportWidget::FindActiveRoadType(road, 2.0), ast::RoadType::kBicycle);

  // Case 6: Match between 2.0 and 5.0 -> kBicycle
  EXPECT_EQ(ViewportWidget::FindActiveRoadType(road, 3.5), ast::RoadType::kBicycle);

  // Case 7: Exact match at s = 5.0 -> kTownExpressway
  EXPECT_EQ(ViewportWidget::FindActiveRoadType(road, 5.0), ast::RoadType::kTownExpressway);

  // Case 8: Match after last record -> kTownExpressway
  EXPECT_EQ(ViewportWidget::FindActiveRoadType(road, 8.0), ast::RoadType::kTownExpressway);
}

TEST(VisTest, BatchMapGeometrySignals) {
  // Arrange
  ast::AbstractSyntaxTree map;

  ast::Road road;
  road.id = "1";
  road.length = 10.0;

  ast::GeometryRecord geom;
  geom.s = 0.0;
  geom.length = 10.0;
  geom.x = 0.0;
  geom.y = 0.0;
  geom.hdg = 0.0;
  geom.shape = ast::Line{};
  road.plan_view.push_back(geom);

  ast::LaneSection section;
  section.s = 0.0;
  ast::Lane lane0;
  lane0.id = 0;
  lane0.type = strada::ast::LaneType::kBorder;
  section.center.push_back(lane0);
  road.lanes.sections.push_back(section);

  // Add a signal with width & height (should draw rectangular face: 4 lines * 2 = 8 vertices + 2 pole vertices = 10
  // total)
  ast::Signal signal;
  signal.id = "signal_1";
  signal.s = 2.0;
  signal.t = 1.0;
  signal.z_offset = 3.0;
  signal.width = 1.0;
  signal.height = 0.5;
  road.signals.push_back(signal);

  // Add a signal reference (should draw circular face: 12 segments * 2 = 24 vertices + 2 pole vertices = 26 total)
  ast::SignalReference sig_ref;
  sig_ref.id = "signal_1";
  sig_ref.s = 5.0;
  sig_ref.t = -1.0;
  sig_ref.z_offset = 2.0;
  road.signal_references.push_back(sig_ref);

  map.roads.push_back(road);

  cpm::CompiledPhysicsModel cpm(map);
  tess::Tessellator tess(map, cpm, 0.5);

  // Act
  auto batched = BatchMapGeometry(tess);

  // Assert
  // Total expected vertices = 10 (for signal) + 26 (for signal reference) = 36 vertices.
  ASSERT_EQ(batched.signal_line_vertices.size(), 36);

  // Color: Electric Cyan/Teal (rgb(0, 229, 255))
  for (const auto& v : batched.signal_line_vertices) {
    EXPECT_NEAR(v.r, 0.0F, 1e-4F);
    EXPECT_NEAR(v.g, 229.0F / 255.0F, 1e-4F);
    EXPECT_NEAR(v.b, 1.0F, 1e-4F);
  }

  // Verify pole bottom and top for signal 1: starts at index 0
  const auto& p1_bottom = batched.signal_line_vertices[0];
  const auto& p1_top = batched.signal_line_vertices[1];
  EXPECT_NEAR(p1_bottom.z, 0.0F, 1e-4F);
  EXPECT_NEAR(p1_top.z, 3.0F, 1e-4F);
}

struct TestViewportWidget : public ViewportWidget {
  using ViewportWidget::keyPressEvent;
  using ViewportWidget::mousePressEvent;
  using ViewportWidget::mouseReleaseEvent;
};

TEST(VisTest, RouteCreationModeToggle) {
  if (!qApp) {
    static int argc = 1;
    static char* argv[] = {const_cast<char*>("test")};
    static QApplication app(argc, argv);
  }

  TestViewportWidget widget;
  EXPECT_FALSE(widget.IsRouteCreationMode());

  // Toggle ON
  QKeyEvent press_p(QEvent::KeyPress, Qt::Key_P, Qt::NoModifier);
  widget.keyPressEvent(&press_p);
  EXPECT_TRUE(widget.IsRouteCreationMode());

  // Toggle OFF
  widget.keyPressEvent(&press_p);
  EXPECT_FALSE(widget.IsRouteCreationMode());
}

TEST(VisTest, ClickVsDragClassification) {
  if (!qApp) {
    static int argc = 1;
    static char* argv[] = {const_cast<char*>("test")};
    static QApplication app(argc, argv);
  }

  TestViewportWidget widget;

  // Enable route creation mode
  QKeyEvent press_p(QEvent::KeyPress, Qt::Key_P, Qt::NoModifier);
  widget.keyPressEvent(&press_p);
  ASSERT_TRUE(widget.IsRouteCreationMode());

  // Press at (100, 100)
  QMouseEvent press_event(QEvent::MouseButtonPress, QPointF(100.0, 100.0), QPointF(100.0, 100.0), Qt::LeftButton,
                          Qt::LeftButton, Qt::NoModifier);
  widget.mousePressEvent(&press_event);

  // Release at (102, 102) -> distance is sqrt(8) ~ 2.82 <= 5.0 -> Click!
  QMouseEvent release_click(QEvent::MouseButtonRelease, QPointF(102.0, 102.0), QPointF(102.0, 102.0), Qt::LeftButton,
                            Qt::NoButton, Qt::NoModifier);
  widget.mouseReleaseEvent(&release_click);
  EXPECT_TRUE(widget.Waypoints().empty());

  // Press again
  widget.mousePressEvent(&press_event);

  // Release at (110, 110) -> distance is sqrt(200) ~ 14.14 > 5.0 -> Drag (ignored as click)!
  QMouseEvent release_drag(QEvent::MouseButtonRelease, QPointF(110.0, 110.0), QPointF(110.0, 110.0), Qt::LeftButton,
                           Qt::NoButton, Qt::NoModifier);
  widget.mouseReleaseEvent(&release_drag);
  EXPECT_TRUE(widget.Waypoints().empty());
}

TEST(VisTest, WaypointSnappingAndShortcuts) {
  if (!qApp) {
    static int argc = 1;
    static char* argv[] = {const_cast<char*>("test")};
    static QApplication app(argc, argv);
  }

  // Arrange: programmatically construct AST with Road 1 (drivable right, non-drivable left)
  ast::AbstractSyntaxTree map;

  ast::Road road;
  road.id = "1";
  road.length = 10.0;

  ast::GeometryRecord geom;
  geom.s = 0.0;
  geom.length = 10.0;
  geom.x = 0.0;
  geom.y = 0.0;
  geom.hdg = 0.0;
  geom.shape = ast::Line{};
  road.plan_view.push_back(geom);

  ast::LaneSection section;
  section.s = 0.0;

  // Drivable right lane (id = -1, type = driving)
  ast::Lane lane_right;
  lane_right.id = -1;
  lane_right.type = strada::ast::LaneType::kDriving;
  ast::LaneWidth w_right;
  w_right.s_offset = 0.0;
  w_right.a = 3.0;
  lane_right.widths.push_back(w_right);
  section.right.push_back(lane_right);

  // Non-drivable left lane (id = 1, type = sidewalk)
  ast::Lane lane_left;
  lane_left.id = 1;
  lane_left.type = strada::ast::LaneType::kSidewalk;
  ast::LaneWidth w_left;
  w_left.s_offset = 0.0;
  w_left.a = 3.0;
  lane_left.widths.push_back(w_left);
  section.left.push_back(lane_left);

  ast::Lane lane0;
  lane0.id = 0;
  lane0.type = strada::ast::LaneType::kBorder;
  section.center.push_back(lane0);

  road.lanes.sections.push_back(section);
  map.roads.push_back(road);

  cpm::CompiledPhysicsModel cpm(map);
  tess::Tessellator tess(map, cpm, 0.5);
  auto batched = BatchMapGeometry(tess);

  TestViewportWidget widget;
  widget.SetGeometry(batched, map, std::move(cpm));

  // Enable Route Creation Mode
  QKeyEvent press_p(QEvent::KeyPress, Qt::Key_P, Qt::NoModifier);
  widget.keyPressEvent(&press_p);
  ASSERT_TRUE(widget.IsRouteCreationMode());

  // Click on non-drivable left lane (sidewalk: y > 0). Let's project world (5.0, 1.5) to screen.
  const auto screen_pos_left = widget.GetCamera().WorldToScreen(5.0F, 1.5F);
  QMouseEvent press_event_left(QEvent::MouseButtonPress, screen_pos_left, screen_pos_left, Qt::LeftButton,
                               Qt::LeftButton, Qt::NoModifier);
  QMouseEvent release_event_left(QEvent::MouseButtonRelease, screen_pos_left, screen_pos_left, Qt::LeftButton,
                                 Qt::NoButton, Qt::NoModifier);

  widget.mousePressEvent(&press_event_left);
  widget.mouseReleaseEvent(&release_event_left);

  // Non-drivable lane should be ignored
  EXPECT_TRUE(widget.Waypoints().empty());

  // Click on drivable right lane (driving: y < 0). Project world (5.0, -1.5) to screen.
  const auto screen_pos_right = widget.GetCamera().WorldToScreen(5.0F, -1.5F);
  QMouseEvent press_event_right(QEvent::MouseButtonPress, screen_pos_right, screen_pos_right, Qt::LeftButton,
                                Qt::LeftButton, Qt::NoModifier);
  QMouseEvent release_event_right(QEvent::MouseButtonRelease, screen_pos_right, screen_pos_right, Qt::LeftButton,
                                  Qt::NoButton, Qt::NoModifier);

  widget.mousePressEvent(&press_event_right);
  widget.mouseReleaseEvent(&release_event_right);

  // Drivable lane should snap and add road ID "1" as a waypoint
  ASSERT_EQ(widget.Waypoints().size(), 1);
  EXPECT_EQ(widget.Waypoints()[0], "1");
  ASSERT_EQ(widget.WaypointCoords().size(), 1);
  EXPECT_NEAR(widget.WaypointCoords()[0].x(), 5.0, 1e-4);
  EXPECT_NEAR(widget.WaypointCoords()[0].y(), -1.5, 1e-4);

  // Click again on drivable right lane to add a second waypoint
  widget.mousePressEvent(&press_event_right);
  widget.mouseReleaseEvent(&release_event_right);
  ASSERT_EQ(widget.Waypoints().size(), 2);
  ASSERT_EQ(widget.WaypointCoords().size(), 2);

  // Undo (Backspace) should remove the last waypoint
  QKeyEvent press_backspace(QEvent::KeyPress, Qt::Key_Backspace, Qt::NoModifier);
  widget.keyPressEvent(&press_backspace);
  EXPECT_EQ(widget.Waypoints().size(), 1);
  EXPECT_EQ(widget.WaypointCoords().size(), 1);

  // Clear (C) should clear all waypoints
  QKeyEvent press_c(QEvent::KeyPress, Qt::Key_C, Qt::NoModifier);
  widget.keyPressEvent(&press_c);
  EXPECT_TRUE(widget.Waypoints().empty());
  EXPECT_TRUE(widget.WaypointCoords().empty());

  // Escape (Escape) should exit Route Creation Mode
  QKeyEvent press_escape(QEvent::KeyPress, Qt::Key_Escape, Qt::NoModifier);
  widget.keyPressEvent(&press_escape);
  EXPECT_FALSE(widget.IsRouteCreationMode());
}

TEST(VisTest, RouteCoordinateMapping) {
  // Arrange
  routing::Route route;

  routing::RouteSegment seg1;
  seg1.road_id = "1";
  seg1.forward = true;
  seg1.length = 10.0;

  routing::RouteSegment seg2;
  seg2.road_id = "2";
  seg2.forward = false;
  seg2.length = 15.0;

  route.segments.push_back(seg1);
  route.segments.push_back(seg2);

  // Act & Assert
  // 1. Test translation on forward segment (Road 1)
  // start_s = 0.0, s_local = 3.0, t_local = 1.5
  // Expect: s_route = 3.0, t_route = 1.5
  auto opt1 = route.ToRouteCoordinates("1", 3.0, 1.5);
  ASSERT_TRUE(opt1.has_value());
  EXPECT_DOUBLE_EQ(opt1->first, 3.0);
  EXPECT_DOUBLE_EQ(opt1->second, 1.5);

  // 2. Test translation on backward segment (Road 2)
  // start_s = 10.0, s_local = 4.0, t_local = 1.5
  // Expect: s_route = start_s + (L_seg2 - s_local) = 10.0 + (15.0 - 4.0) = 21.0
  // Expect: t_route = -t_local = -1.5
  auto opt2 = route.ToRouteCoordinates("2", 4.0, 1.5);
  ASSERT_TRUE(opt2.has_value());
  EXPECT_DOUBLE_EQ(opt2->first, 21.0);
  EXPECT_DOUBLE_EQ(opt2->second, -1.5);

  // 3. Test road not in route
  auto opt3 = route.ToRouteCoordinates("3", 5.0, 0.0);
  EXPECT_FALSE(opt3.has_value());
}

TEST(VisTest, RoutePlannerHUDCardAndPathingErrors) {
  if (!qApp) {
    static int argc = 1;
    static char* argv[] = {const_cast<char*>("test")};
    static QApplication app(argc, argv);
  }

  // Arrange: programmatically construct AST with two disconnected roads: Road 1 and Road 2
  ast::AbstractSyntaxTree map;

  // Road 1
  ast::Road road1;
  road1.id = "1";
  road1.length = 10.0;
  ast::GeometryRecord geom1;
  geom1.s = 0.0;
  geom1.length = 10.0;
  geom1.x = 0.0;
  geom1.y = 0.0;
  geom1.hdg = 0.0;
  geom1.shape = ast::Line{};
  road1.plan_view.push_back(geom1);

  ast::LaneSection sec1;
  sec1.s = 0.0;
  ast::Lane lane_right1;
  lane_right1.id = -1;
  lane_right1.type = strada::ast::LaneType::kDriving;
  ast::LaneWidth w_right1;
  w_right1.s_offset = 0.0;
  w_right1.a = 3.0;
  lane_right1.widths.push_back(w_right1);
  sec1.right.push_back(lane_right1);
  ast::Lane lane0_1;
  lane0_1.id = 0;
  lane0_1.type = strada::ast::LaneType::kBorder;
  sec1.center.push_back(lane0_1);
  road1.lanes.sections.push_back(sec1);
  map.roads.push_back(road1);

  // Road 2 (disconnected, offset in y direction)
  ast::Road road2;
  road2.id = "2";
  road2.length = 10.0;
  ast::GeometryRecord geom2;
  geom2.s = 0.0;
  geom2.length = 10.0;
  geom2.x = 0.0;
  geom2.y = 20.0;
  geom2.hdg = 0.0;
  geom2.shape = ast::Line{};
  road2.plan_view.push_back(geom2);

  ast::LaneSection sec2;
  sec2.s = 0.0;
  ast::Lane lane_right2;
  lane_right2.id = -1;
  lane_right2.type = strada::ast::LaneType::kDriving;
  ast::LaneWidth w_right2;
  w_right2.s_offset = 0.0;
  w_right2.a = 3.0;
  lane_right2.widths.push_back(w_right2);
  sec2.right.push_back(lane_right2);
  ast::Lane lane0_2;
  lane0_2.id = 0;
  lane0_2.type = strada::ast::LaneType::kBorder;
  sec2.center.push_back(lane0_2);
  road2.lanes.sections.push_back(sec2);
  map.roads.push_back(road2);

  cpm::CompiledPhysicsModel cpm(map);
  tess::Tessellator tess(map, cpm, 0.5);
  auto batched = BatchMapGeometry(tess);

  TestViewportWidget widget;
  widget.SetGeometry(batched, map, std::move(cpm));

  // Enable Route Creation Mode
  QKeyEvent press_p(QEvent::KeyPress, Qt::Key_P, Qt::NoModifier);
  widget.keyPressEvent(&press_p);
  ASSERT_TRUE(widget.IsRouteCreationMode());

  // Click on Road 1 (5.0, -1.5)
  const auto screen_pos1 = widget.GetCamera().WorldToScreen(5.0F, -1.5F);
  QMouseEvent press_event1(QEvent::MouseButtonPress, screen_pos1, screen_pos1, Qt::LeftButton, Qt::LeftButton,
                           Qt::NoModifier);
  QMouseEvent release_event1(QEvent::MouseButtonRelease, screen_pos1, screen_pos1, Qt::LeftButton, Qt::NoButton,
                             Qt::NoModifier);
  widget.mousePressEvent(&press_event1);
  widget.mouseReleaseEvent(&release_event1);

  // Click on Road 2 (5.0, 18.5)
  const auto screen_pos2 = widget.GetCamera().WorldToScreen(5.0F, 18.5F);
  QMouseEvent press_event2(QEvent::MouseButtonPress, screen_pos2, screen_pos2, Qt::LeftButton, Qt::LeftButton,
                           Qt::NoModifier);
  QMouseEvent release_event2(QEvent::MouseButtonRelease, screen_pos2, screen_pos2, Qt::LeftButton, Qt::NoButton,
                             Qt::NoModifier);
  widget.mousePressEvent(&press_event2);
  widget.mouseReleaseEvent(&release_event2);

  // Act & Assert
  // Verify waypoints are added
  ASSERT_EQ(widget.Waypoints().size(), 2);
  EXPECT_EQ(widget.Waypoints()[0], "1");
  EXPECT_EQ(widget.Waypoints()[1], "2");

  // Since roads 1 and 2 are completely disconnected, Dijkstra should fail
  EXPECT_FALSE(widget.ActiveRoute().has_value());
  EXPECT_EQ(widget.RouteError(), "No path found between road 1 and 2");
}

}  // namespace strada::vis
