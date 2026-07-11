// SPDX-License-Identifier: BSL-1.0

#include <gtest/gtest.h>

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

  auto cpm = cpm::CompiledPhysicsModel::Build(map);
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

  auto cpm = cpm::CompiledPhysicsModel::Build(map);
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

  auto cpm = cpm::CompiledPhysicsModel::Build(map);
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
  auto cpm = cpm::CompiledPhysicsModel::Build(map);
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

  cpm::CompiledPhysicsModel cpm = cpm::CompiledPhysicsModel::Build(map);
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

  cpm::CompiledPhysicsModel cpm = cpm::CompiledPhysicsModel::Build(map);
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

}  // namespace strada::vis
