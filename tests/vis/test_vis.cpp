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

  tess::Tessellator tess(map, 0.5);
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

  tess::Tessellator tess(map, 0.5);
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

  tess::Tessellator tess(map, 0.5);
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
  tess::Tessellator tess(map, 0.5);
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

TEST(VisTest, FindActiveRoadType) {
  // Arrange
  ast::Road road;
  road.id = "1";
  road.length = 10.0;

  // Case 1: Empty types vector
  EXPECT_EQ(ViewportWidget::FindActiveRoadType(road, 0.0), ast::RoadType::kUnknown);
  EXPECT_EQ(ViewportWidget::FindActiveRoadType(road, 5.0), ast::RoadType::kUnknown);

  // Set up types
  road.types.push_back(ast::RoadTypeRecord{0.0, ast::RoadType::kTownLocal});
  road.types.push_back(ast::RoadTypeRecord{2.0, ast::RoadType::kBicycle});
  road.types.push_back(ast::RoadTypeRecord{5.0, ast::RoadType::kTownExpressway});

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

}  // namespace strada::vis
