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
  auto tess = BuildTessellator(map, 0.5);

  // Assert: no meshes or polylines
  EXPECT_TRUE(tess.Meshes().empty());
  EXPECT_TRUE(tess.Polylines().empty());
}

TEST(TessellatorTest, StraightRoadReferenceLine) {
  // Arrange: parse roads.xodr containing two straight line roads
  auto map_path = GetTestDataPath("roads.xodr");
  auto map = parser::ParseFile(map_path);

  // Act: tessellate with 0.5m chord error
  auto tess = BuildTessellator(map, 0.5);

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
  EXPECT_NEAR(road1_ref.vertices.front().x, 0.0f, 1e-3f);
  EXPECT_NEAR(road1_ref.vertices.front().y, 0.0f, 1e-3f);
  EXPECT_NEAR(road1_ref.vertices.front().z, 0.0f, 1e-3f);

  // Last point should be at (10, 0, 0)
  EXPECT_NEAR(road1_ref.vertices.back().x, 10.0f, 1e-3f);
  EXPECT_NEAR(road1_ref.vertices.back().y, 0.0f, 1e-3f);
  EXPECT_NEAR(road1_ref.vertices.back().z, 0.0f, 1e-3f);

  // Road 2 (length 25.5) reference line properties
  const auto& road2_ref = ref_lines[1];
  EXPECT_EQ(road2_ref.road_id, static_cast<cpm::RoadId>(1));
  ASSERT_GE(road2_ref.vertices.size(), 2);

  EXPECT_NEAR(road2_ref.vertices.front().x, 0.0f, 1e-3f);
  EXPECT_NEAR(road2_ref.vertices.front().y, 0.0f, 1e-3f);

  // Last point should be at (25.5, 0, 0)
  EXPECT_NEAR(road2_ref.vertices.back().x, 25.5f, 1e-3f);
  EXPECT_NEAR(road2_ref.vertices.back().y, 0.0f, 1e-3f);
}

TEST(TessellatorTest, LaneBoundariesAndMarkingTypes) {
  // Arrange: parse lanes_flat.xodr containing a multi-lane road
  auto map_path = GetTestDataPath("lanes_flat.xodr");
  auto map = parser::ParseFile(map_path);

  // Act: tessellate
  auto tess = BuildTessellator(map, 0.5);

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
  EXPECT_NEAR(left_boundary.vertices.front().x, 0.0f, 1e-2f);
  EXPECT_NEAR(left_boundary.vertices.front().y, 3.5f, 1e-2f);

  // Right lane (id -1)
  const auto& right_boundary = (boundaries[0].original_lane_id == -1) ? boundaries[0] : boundaries[1];
  EXPECT_EQ(right_boundary.original_lane_id, -1);
  EXPECT_EQ(right_boundary.marking_type, "solid");
  EXPECT_FALSE(right_boundary.is_reference_line);
  ASSERT_GE(right_boundary.vertices.size(), 2);

  // At s=0, laneOffset=0.5. Right lane width at sOffset=1.0 is 3.2 with linear slope 0.2, which evaluates to 3.0 at s=0.
  // Thus, the outer boundary of the right lane (id -1) is at 0.5 - 3.0 = -2.5.
  EXPECT_NEAR(right_boundary.vertices.front().x, 0.0f, 1e-2f);
  EXPECT_NEAR(right_boundary.vertices.front().y, -2.5f, 1e-2f);
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
  lane1.type = "driving";
  ast::LaneWidth w1;
  w1.s_offset = 0.0;
  w1.a = 3.0;
  lane1.widths.push_back(w1);
  section.left.push_back(lane1);
  
  // Left lane 2
  ast::Lane lane2;
  lane2.id = 2;
  lane2.type = "driving";
  ast::LaneWidth w2;
  w2.s_offset = 0.0;
  w2.a = 3.5;
  lane2.widths.push_back(w2);
  section.left.push_back(lane2);
  
  // Center lane 0
  ast::Lane lane0;
  lane0.id = 0;
  lane0.type = "border";
  section.center.push_back(lane0);
  
  road.lanes.sections.push_back(section);
  map.roads.push_back(road);
  
  // Act: tessellate
  auto tess = BuildTessellator(map, 0.5);
  
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

}  // namespace strada::tess

