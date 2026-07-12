// SPDX-License-Identifier: BSL-1.0

#include <gtest/gtest.h>

#include <filesystem>
#include <strada/ast/abstract_syntax_tree.hpp>
#include <strada/cpm/compiled_physics_model.hpp>
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
  cpm::CompiledPhysicsModel model(map);

  // Act: tessellate
  Tessellator tess(map, model, 0.5);

  // Assert: no meshes or polylines
  EXPECT_TRUE(tess.Meshes().empty());
  EXPECT_TRUE(tess.Polylines().empty());
}

TEST(TessellatorTest, StraightRoadReferenceLine) {
  // Arrange: parse roads.xodr containing two straight line roads
  auto map_path = GetTestDataPath("roads.xodr");
  auto map = parser::ParseFile(map_path);
  cpm::CompiledPhysicsModel model(map);

  // Act: tessellate with 0.5m chord error
  Tessellator tess(map, model, 0.5);

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
  cpm::CompiledPhysicsModel model(map);

  // Act: tessellate
  Tessellator tess(map, model, 0.5);

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
  cpm::CompiledPhysicsModel model(map);

  // Act: tessellate
  Tessellator tess(map, model, 0.5);

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
  cpm::CompiledPhysicsModel model(map);

  // Act: tessellate
  Tessellator tess(map, model, 0.5);

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
  cpm::CompiledPhysicsModel model(map);

  // Act
  Tessellator tess(map, model, 0.5);

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
  cpm::CompiledPhysicsModel model(map);

  // Act
  Tessellator tess(map, model, 0.5);

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
  cpm::CompiledPhysicsModel model(map);

  // Act
  Tessellator tess(map, model, 0.5);

  // Assert
  const auto& boundaries = tess.JunctionBoundaries();
  ASSERT_EQ(boundaries.size(), 1);
  const auto& b = boundaries[0];
  EXPECT_EQ(b.junction_id, "100");
  EXPECT_FALSE(b.vertices.empty());
  EXPECT_FALSE(b.indices.empty());
}

TEST(TessellatorTest, RoadObjectsTessellation) {
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

  // 1. Add object with length/width > 0 (Oriented 2D Box)
  ast::Object obj_box;
  obj_box.id = "obj_box";
  obj_box.s = 5.0;
  obj_box.t = 0.0;
  obj_box.z_offset = 1.0;
  obj_box.length = 2.0;
  obj_box.width = 1.0;
  obj_box.hdg = 0.0;
  obj_box.pitch = 0.0;
  obj_box.roll = 0.0;
  road.objects.push_back(obj_box);

  // 2. Add other object (Crosshair)
  ast::Object obj_cross;
  obj_cross.id = "obj_cross";
  obj_cross.s = 8.0;
  obj_cross.t = 0.0;
  obj_cross.z_offset = 0.0;
  road.objects.push_back(obj_cross);

  map.roads.push_back(road);

  cpm::CompiledPhysicsModel model(map);

  // Act
  Tessellator tess(map, model, 0.5);

  // Assert
  const auto& objects = tess.Objects();
  ASSERT_EQ(objects.size(), 2);

  // 1. Check obj_box
  const auto& o_box = (objects[0].id == "obj_box") ? objects[0] : objects[1];
  EXPECT_EQ(o_box.id, "obj_box");
  ASSERT_EQ(o_box.outlines.size(), 1);
  // Box has 4 corners + closed loop -> 5 vertices
  EXPECT_EQ(o_box.outlines[0].size(), 5);

  // Check coordinates of the box corners (s=5.0, length=2.0, width=1.0, z_offset=1.0)
  // local corners relative to obj center:
  // (1.0, 0.5, 0.0), (1.0, -0.5, 0.0), (-1.0, -0.5, 0.0), (-1.0, 0.5, 0.0)
  // since road is along X-axis, world corners should be:
  // (6.0, 0.5, 1.0), (6.0, -0.5, 1.0), (4.0, -0.5, 1.0), (4.0, 0.5, 1.0)
  const auto& box_pts = o_box.outlines[0];
  EXPECT_NEAR(box_pts[0].x, 6.0F, 1e-3F);
  EXPECT_NEAR(box_pts[0].y, 0.5F, 1e-3F);
  EXPECT_NEAR(box_pts[0].z, 1.0F, 1e-3F);

  EXPECT_NEAR(box_pts[4].x, 6.0F, 1e-3F);
  EXPECT_NEAR(box_pts[4].y, 0.5F, 1e-3F);
  EXPECT_NEAR(box_pts[4].z, 1.0F, 1e-3F);

  // 2. Check obj_cross
  const auto& o_cross = (objects[0].id == "obj_cross") ? objects[0] : objects[1];
  EXPECT_EQ(o_cross.id, "obj_cross");
  // Crosshair has 2 separate line segment outlines
  ASSERT_EQ(o_cross.outlines.size(), 2);
  EXPECT_EQ(o_cross.outlines[0].size(), 2);
  EXPECT_EQ(o_cross.outlines[1].size(), 2);
}

TEST(TessellatorTest, RoadObjectsCornersTessellation) {
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
  outline_road.corners_road.push_back(ast::ObjectCornerRoad{.s = 4.0, .t = -1.0, .dz = 0.5});
  outline_road.corners_road.push_back(ast::ObjectCornerRoad{.s = 5.0, .t = 1.0, .dz = 0.5});
  obj_road.outlines.push_back(outline_road);
  road.objects.push_back(obj_road);

  map.roads.push_back(road);

  cpm::CompiledPhysicsModel model(map);

  // Act
  Tessellator tess(map, model, 0.5);

  // Assert
  const auto& objects = tess.Objects();
  ASSERT_EQ(objects.size(), 2);

  // 1. Check obj_local
  const auto& o_local = (objects[0].id == "obj_local") ? objects[0] : objects[1];
  EXPECT_EQ(o_local.id, "obj_local");
  ASSERT_EQ(o_local.outlines.size(), 1);
  // Closed loop: 4 corners + 1 duplicate -> 5 vertices
  EXPECT_EQ(o_local.outlines[0].size(), 5);

  // check coordinates: center at (2,0,1). Heading 0.
  // corner (-1, -1, 0) -> (2 - 1, 0 - 1, 1 + 0) = (1, -1, 1)
  const auto& local_pts = o_local.outlines[0];
  EXPECT_NEAR(local_pts[0].x, 1.0F, 1e-3F);
  EXPECT_NEAR(local_pts[0].y, -1.0F, 1e-3F);
  EXPECT_NEAR(local_pts[0].z, 1.0F, 1e-3F);

  // 2. Check obj_road
  const auto& o_road = (objects[0].id == "obj_road") ? objects[0] : objects[1];
  EXPECT_EQ(o_road.id, "obj_road");
  ASSERT_EQ(o_road.outlines.size(), 1);
  // Open loop: 2 corners -> 2 vertices
  EXPECT_EQ(o_road.outlines[0].size(), 2);

  // corner 1: s=4.0, t=-1.0, dz=0.5 -> world (4.0, -1.0, 0.5)
  const auto& road_pts = o_road.outlines[0];
  EXPECT_NEAR(road_pts[0].x, 4.0F, 1e-3F);
  EXPECT_NEAR(road_pts[0].y, -1.0F, 1e-3F);
  EXPECT_NEAR(road_pts[0].z, 0.5F, 1e-3F);

  // corner 2: s=5.0, t=1.0, dz=0.5 -> world (5.0, 1.0, 0.5)
  EXPECT_NEAR(road_pts[1].x, 5.0F, 1e-3F);
  EXPECT_NEAR(road_pts[1].y, 1.0F, 1e-3F);
  EXPECT_NEAR(road_pts[1].z, 0.5F, 1e-3F);
}

TEST(TessellatorTest, RoadSignalsTessellation) {
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
  sig_ref.id = "signal_ref_1";
  sig_ref.s = 5.0;
  sig_ref.t = -1.0;
  sig_ref.z_offset = 2.0;
  road.signal_references.push_back(sig_ref);

  map.roads.push_back(road);

  cpm::CompiledPhysicsModel model(map);

  // Act
  Tessellator tess(map, model, 0.5);

  // Assert
  const auto& signals = tess.Signals();
  ASSERT_EQ(signals.size(), 2);

  // 1. Check signal_1
  const auto& signal_1_tess = (signals[0].id == "signal_1") ? signals[0] : signals[1];
  EXPECT_EQ(signal_1_tess.id, "signal_1");
  ASSERT_EQ(signal_1_tess.outlines.size(), 2);
  // Pole has 2 vertices
  EXPECT_EQ(signal_1_tess.outlines[0].size(), 2);
  // Board has 4 corners + closed loop -> 5 vertices
  EXPECT_EQ(signal_1_tess.outlines[1].size(), 5);

  // Pole bottom should be at road level z=0, pole top at z_offset=3.0
  EXPECT_NEAR(signal_1_tess.outlines[0][0].x, 2.0F, 1e-3F);
  EXPECT_NEAR(signal_1_tess.outlines[0][0].y, 1.0F, 1e-3F);
  EXPECT_NEAR(signal_1_tess.outlines[0][0].z, 0.0F, 1e-3F);

  EXPECT_NEAR(signal_1_tess.outlines[0][1].x, 2.0F, 1e-3F);
  EXPECT_NEAR(signal_1_tess.outlines[0][1].y, 1.0F, 1e-3F);
  EXPECT_NEAR(signal_1_tess.outlines[0][1].z, 3.0F, 1e-3F);

  // 2. Check signal_ref_1
  const auto& signal_ref_tess = (signals[0].id == "signal_ref_1") ? signals[0] : signals[1];
  EXPECT_EQ(signal_ref_tess.id, "signal_ref_1");
  ASSERT_EQ(signal_ref_tess.outlines.size(), 2);
  // Pole has 2 vertices
  EXPECT_EQ(signal_ref_tess.outlines[0].size(), 2);
  // Circle board has 12 segments + closed loop -> 13 vertices
  EXPECT_EQ(signal_ref_tess.outlines[1].size(), 13);
}

}  // namespace strada::tess
