// SPDX-License-Identifier: BSL-1.0

#include <gtest/gtest.h>

#include <filesystem>
#include <strada/ast/abstract_syntax_tree.hpp>
#include <strada/parser/parser.hpp>
#include <strada/tess/tessellator.hpp>

#ifndef STRADA_TEST_DATA_DIR
#define STRADA_TEST_DATA_DIR "tests/data"
#endif

namespace strada::tess {

// Helper to construct absolute path to test fixtures
static auto GetTestDataPath(const std::string& filename) -> std::filesystem::path {
  return std::filesystem::path(STRADA_TEST_DATA_DIR) / filename;
}

TEST(TessellatorTest, EmptyMap) {
  // Arrange: empty AST
  ast::AbstractSyntaxTree map;

  // Act: tessellate
  Tessellator tess(map, 0.5);

  // Assert: no meshes or polylines
  EXPECT_TRUE(tess.Meshes().empty());
  EXPECT_TRUE(tess.Polylines().empty());
}

TEST(TessellatorTest, StraightRoadReferenceLine) {
  // Arrange: parse roads.xodr containing two straight line roads
  auto map_path = GetTestDataPath("roads.xodr");
  auto map = parser::ParseFile(map_path);

  // Act: tessellate with 0.5m chord error
  Tessellator tess(map, 0.5);

  // Assert: we expect 2 reference lines (one per road)
  // Let's filter polylines representing reference lines
  std::vector<Polyline> ref_lines;
  for (const auto& poly : tess.Polylines()) {
    if (poly.is_reference_line) {
      ref_lines.push_back(poly);
    }
  }

  ASSERT_EQ(ref_lines.size(), 2);

  // Road 1 (length 10.0) reference line properties
  const auto& road1_ref = ref_lines[0];
  EXPECT_EQ(road1_ref.road_id, static_cast<cpm::RoadId>(0));
  EXPECT_EQ(road1_ref.marking_type, "solid");
  ASSERT_GE(road1_ref.vertices.size(), 2);

  // First point should be at (0, 0, 0)
  EXPECT_NEAR(road1_ref.vertices.front().x, 0.0F, 1e-3F);
  EXPECT_NEAR(road1_ref.vertices.front().y, 0.0F, 1e-3F);
  EXPECT_NEAR(road1_ref.vertices.front().z, 0.0F, 1e-3F);

  // Last point should be at (10, 0, 0)
  EXPECT_NEAR(road1_ref.vertices.back().x, 10.0F, 1e-3F);
  EXPECT_NEAR(road1_ref.vertices.back().y, 0.0F, 1e-3F);
  EXPECT_NEAR(road1_ref.vertices.back().z, 0.0F, 1e-3F);

  // Road 2 (length 25.5) reference line properties
  const auto& road2_ref = ref_lines[1];
  EXPECT_EQ(road2_ref.road_id, static_cast<cpm::RoadId>(1));
  ASSERT_GE(road2_ref.vertices.size(), 2);

  EXPECT_NEAR(road2_ref.vertices.front().x, 0.0F, 1e-3F);
  EXPECT_NEAR(road2_ref.vertices.front().y, 0.0F, 1e-3F);

  // Last point should be at (25.5, 0, 0)
  EXPECT_NEAR(road2_ref.vertices.back().x, 25.5F, 1e-3F);
  EXPECT_NEAR(road2_ref.vertices.back().y, 0.0F, 1e-3F);
}

TEST(TessellatorTest, LaneBoundariesAndMarkingTypes) {
  // Arrange: parse lanes_flat.xodr containing a multi-lane road
  auto map_path = GetTestDataPath("lanes_flat.xodr");
  auto map = parser::ParseFile(map_path);

  // Act: tessellate
  Tessellator tess(map, 0.5);

  // Assert:
  std::vector<Polyline> boundaries;
  for (const auto& poly : tess.Polylines()) {
    if (!poly.is_reference_line) {
      boundaries.push_back(poly);
    }
  }

  // lanes_flat.xodr has one left lane (1) and one right lane (-1)
  ASSERT_EQ(boundaries.size(), 2);

  // Left lane (id 1)
  const auto& left_boundary = (boundaries[0].original_lane_id == 1) ? boundaries[0] : boundaries[1];
  EXPECT_EQ(left_boundary.original_lane_id, 1);
  EXPECT_EQ(left_boundary.marking_type, "solid");
  EXPECT_FALSE(left_boundary.is_reference_line);
  ASSERT_GE(left_boundary.vertices.size(), 2);

  // At s=0, laneOffset=0.5. Left lane width = 3.0. Outer boundary is at 0.5 + 3.0 = 3.5.
  EXPECT_NEAR(left_boundary.vertices.front().x, 0.0F, 1e-2F);
  EXPECT_NEAR(left_boundary.vertices.front().y, 3.5F, 1e-2F);

  // Right lane (id -1)
  const auto& right_boundary = (boundaries[0].original_lane_id == -1) ? boundaries[0] : boundaries[1];
  EXPECT_EQ(right_boundary.original_lane_id, -1);
  EXPECT_EQ(right_boundary.marking_type, "solid");
  EXPECT_FALSE(right_boundary.is_reference_line);
  ASSERT_GE(right_boundary.vertices.size(), 2);

  // At s=0, laneOffset=0.5. Right lane width at sOffset=1.0 is 3.2 with linear slope 0.2, which evaluates to 3.0 at
  // s=0. Thus, the outer boundary of the right lane (id -1) is at 0.5 - 3.0 = -2.5.
  EXPECT_NEAR(right_boundary.vertices.front().x, 0.0F, 1e-2F);
  EXPECT_NEAR(right_boundary.vertices.front().y, -2.5F, 1e-2F);
}

TEST(TessellatorTest, MultipleLanesMarkingTypes) {
  // Arrange: construct an AST programmatically with a road containing lanes 1 and 2
  ast::AbstractSyntaxTree map;

  ast::Road road;
  road.id = "1";
  road.length = 10.0;

  // Set up plan view
  ast::GeometryRecord geom;
  geom.s = 0.0;
  geom.length = 10.0;
  geom.x = 0.0;
  geom.y = 0.0;
  geom.hdg = 0.0;
  geom.shape = ast::Line{};
  road.plan_view.push_back(geom);

  // Set up lanes
  ast::LaneSection section;
  section.s = 0.0;

  // Left lane 1
  ast::Lane lane1;
  lane1.id = 1;
  lane1.type = strada::ast::LaneType::kDriving;
  ast::LaneWidth w1;
  w1.s_offset = 0.0;
  w1.a = 3.0;
  lane1.widths.push_back(w1);
  section.left.push_back(lane1);

  // Left lane 2
  ast::Lane lane2;
  lane2.id = 2;
  lane2.type = strada::ast::LaneType::kDriving;
  ast::LaneWidth w2;
  w2.s_offset = 0.0;
  w2.a = 3.5;
  lane2.widths.push_back(w2);
  section.left.push_back(lane2);

  // Center lane 0
  ast::Lane lane0;
  lane0.id = 0;
  lane0.type = strada::ast::LaneType::kBorder;
  section.center.push_back(lane0);

  road.lanes.sections.push_back(section);
  map.roads.push_back(road);

  // Act: tessellate
  Tessellator tess(map, 0.5);

  // Assert: we expect two lane boundaries (for lane 1 and lane 2) + center reference line
  std::vector<Polyline> boundaries;
  for (const auto& poly : tess.Polylines()) {
    if (!poly.is_reference_line) {
      boundaries.push_back(poly);
    }
  }

  ASSERT_EQ(boundaries.size(), 2);

  const auto& b1 = (boundaries[0].original_lane_id == 1) ? boundaries[0] : boundaries[1];
  const auto& b2 = (boundaries[0].original_lane_id == 2) ? boundaries[0] : boundaries[1];

  EXPECT_EQ(b1.marking_type, "broken");
  EXPECT_EQ(b2.marking_type, "solid");
}

TEST(TessellatorTest, LaneSurfaceTriangulation) {
  // Arrange: parse lanes_flat.xodr
  auto map_path = GetTestDataPath("lanes_flat.xodr");
  auto map = parser::ParseFile(map_path);

  // Act: tessellate
  Tessellator tess(map, 0.5);

  // Assert: we expect 2 meshes
  const auto& meshes = tess.Meshes();
  ASSERT_EQ(meshes.size(), 2);

  for (const auto& mesh : meshes) {
    // Validate metadata
    EXPECT_EQ(mesh.road_id, static_cast<cpm::RoadId>(0));
    EXPECT_EQ(mesh.lane_type, ast::LaneType::kDriving);
    EXPECT_TRUE(mesh.original_lane_id == 1 || mesh.original_lane_id == -1);

    // Vertex and Index size consistency
    ASSERT_FALSE(mesh.vertices.empty());
    ASSERT_FALSE(mesh.indices.empty());

    std::size_t num_stations = mesh.vertices.size() / 2;
    EXPECT_EQ(mesh.vertices.size(), num_stations * 2);
    EXPECT_EQ(mesh.indices.size(), 6 * (num_stations - 1));

    // Verify indices point within valid vertex range
    for (std::uint32_t idx : mesh.indices) {
      EXPECT_LT(idx, mesh.vertices.size());
    }

    // Verify winding order is strictly Counter-Clockwise (CCW) in the xy-plane
    for (std::size_t i = 0; i < mesh.indices.size(); i += 3) {
      std::uint32_t idx0 = mesh.indices[i];
      std::uint32_t idx1 = mesh.indices[i + 1];
      std::uint32_t idx2 = mesh.indices[i + 2];

      const auto& v0 = mesh.vertices[idx0];
      const auto& v1 = mesh.vertices[idx1];
      const auto& v2 = mesh.vertices[idx2];

      // 2D cross product in xy plane
      float cp = ((v1.x - v0.x) * (v2.y - v1.y)) - ((v1.y - v0.y) * (v2.x - v1.x));

      // Since it's CCW, cross product should be positive
      EXPECT_GT(cp, 0.0F) << "Triangle (" << idx0 << ", " << idx1 << ", " << idx2
                          << ") winding not CCW. Cross product is " << cp;
    }
  }
}

TEST(TessellatorTest, JunctionBoundaryTessellation) {
  // Arrange
  auto map_path = GetTestDataPath("junction_boundary.xodr");
  auto map = parser::ParseFile(map_path);

  // Act
  Tessellator tess(map, 0.5);

  // Assert
  const auto& boundaries = tess.JunctionBoundaries();
  ASSERT_EQ(boundaries.size(), 1);
  const auto& b = boundaries[0];
  EXPECT_EQ(b.junction_id, "1");

  ASSERT_FALSE(b.vertices.empty());
  ASSERT_FALSE(b.indices.empty());
}

TEST(TessellatorTest, JunctionJointBoundaryTessellation) {
  // Arrange: programmatically construct an AST with a road and a junction boundary containing joint segments
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

  ast::Lane lane_r1;
  lane_r1.id = -1;
  lane_r1.type = strada::ast::LaneType::kDriving;
  ast::LaneWidth w_r1;
  w_r1.s_offset = 0.0;
  w_r1.a = 3.0;
  lane_r1.widths.push_back(w_r1);
  section.right.push_back(lane_r1);

  ast::Lane lane_r2;
  lane_r2.id = -2;
  lane_r2.type = strada::ast::LaneType::kDriving;
  ast::LaneWidth w_r2;
  w_r2.s_offset = 0.0;
  w_r2.a = 4.0;
  lane_r2.widths.push_back(w_r2);
  section.right.push_back(lane_r2);

  ast::Lane lane_l1;
  lane_l1.id = 1;
  lane_l1.type = strada::ast::LaneType::kDriving;
  ast::LaneWidth w_l1;
  w_l1.s_offset = 0.0;
  w_l1.a = 3.0;
  lane_l1.widths.push_back(w_l1);
  section.left.push_back(lane_l1);

  road.lanes.sections.push_back(section);
  map.roads.push_back(road);

  ast::Junction junction;
  junction.id = "100";

  ast::JunctionBoundary boundary;

  // segment 0: type="joint" roadId="1" contactPoint="end" jointLaneStart="-1" jointLaneEnd="-2"
  ast::JunctionBoundarySegment seg0;
  seg0.type = ast::JunctionSegmentType::kJoint;
  seg0.road_id = "1";
  seg0.contact_point = ast::ContactPoint::kEnd;
  seg0.joint_lane_start = -1;
  seg0.joint_lane_end = -2;
  boundary.segments.push_back(seg0);

  // segment 1: type="joint" roadId="1" contactPoint="start" (fallback to entire road width)
  ast::JunctionBoundarySegment seg1;
  seg1.type = ast::JunctionSegmentType::kJoint;
  seg1.road_id = "1";
  seg1.contact_point = ast::ContactPoint::kStart;
  boundary.segments.push_back(seg1);

  junction.boundary = boundary;
  map.junctions.push_back(junction);

  // Act
  Tessellator tess(map, 0.5);

  // Assert
  const auto& boundaries = tess.JunctionBoundaries();
  ASSERT_EQ(boundaries.size(), 1);
  const auto& b = boundaries[0];
  EXPECT_EQ(b.junction_id, "100");
  EXPECT_FALSE(b.vertices.empty());
  EXPECT_FALSE(b.indices.empty());
}

TEST(TessellatorTest, JunctionBoundaryFallbackWithoutBoundaryTag) {
  // Arrange: programmatically construct an AST with a road inside a junction, but NO junction.boundary
  ast::AbstractSyntaxTree map;

  ast::Road road;
  road.id = "1";
  road.junction = "100";  // belongs to junction 100
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

  ast::Lane lane_r1;
  lane_r1.id = -1;
  lane_r1.type = strada::ast::LaneType::kDriving;
  ast::LaneWidth w_r1;
  w_r1.s_offset = 0.0;
  w_r1.a = 3.0;
  lane_r1.widths.push_back(w_r1);
  section.right.push_back(lane_r1);

  road.lanes.sections.push_back(section);
  map.roads.push_back(road);

  ast::Junction junction;
  junction.id = "100";
  // NO boundary set
  map.junctions.push_back(junction);

  // Act
  Tessellator tess(map, 0.5);

  // Assert
  const auto& boundaries = tess.JunctionBoundaries();
  ASSERT_EQ(boundaries.size(), 1);
  const auto& b = boundaries[0];
  EXPECT_EQ(b.junction_id, "100");
  EXPECT_FALSE(b.vertices.empty());
  EXPECT_FALSE(b.indices.empty());
}

}  // namespace strada::tess
