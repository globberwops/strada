#include <gtest/gtest.h>

#include <cmath>
#include <strada/cpm/reference_line.hpp>
#include <strada/parser/parser.hpp>

namespace strada::cpm {

TEST(ReferenceLineTest, CompileAndEvaluateSimpleLine) {
  // Arrange
  const std::string kXml = R"(<?xml version="1.0" standalone="yes"?>
<OpenDRIVE>
  <header revMajor="1" revMinor="9" name="Test Map" version="1.0" date="2026-06-14T09:00:00" north="100.0" south="-100.0" east="200.0" west="-200.0"/>
  <road name="Road 1" length="50.0" id="1" junction="-1" rule="RHT">
    <planView>
      <geometry s="0.0" x="10.0" y="20.0" hdg="0.5" length="50.0">
        <line/>
      </geometry>
    </planView>
    <lanes>
      <laneSection s="0.0">
        <center>
          <lane id="0"/>
        </center>
      </laneSection>
    </lanes>
  </road>
</OpenDRIVE>)";

  auto ast = strada::parser::ParseString(kXml);

  // Act
  auto ref_line = ReferenceLine::Build(ast);

  // Assert structure
  EXPECT_EQ(ref_line.TotalSegmentsCount(), 1);

  auto [first_idx, count] = ref_line.GetRoadSegments(RoadId{0});
  EXPECT_EQ(first_idx, 0);
  EXPECT_EQ(count, 1);
  EXPECT_DOUBLE_EQ(ref_line.GetSegmentSStart(0), 0.0);
  EXPECT_DOUBLE_EQ(ref_line.GetSegmentLength(0), 50.0);

  // Act & Assert point evaluation
  auto pt_start = ref_line.Evaluate(0, 0.0);
  EXPECT_NEAR(pt_start.x, 10.0, 1e-9);
  EXPECT_NEAR(pt_start.y, 20.0, 1e-9);
  EXPECT_NEAR(pt_start.heading, 0.5, 1e-9);

  auto pt_mid = ref_line.Evaluate(0, 25.0);
  double expected_x = 10.0 + 25.0 * std::cos(0.5);
  double expected_y = 20.0 + 25.0 * std::sin(0.5);
  EXPECT_NEAR(pt_mid.x, expected_x, 1e-9);
  EXPECT_NEAR(pt_mid.y, expected_y, 1e-9);
  EXPECT_NEAR(pt_mid.heading, 0.5, 1e-9);
}

TEST(ReferenceLineTest, CompileAndProjectLineAndArc) {
  // Arrange
  const std::string kXml = R"(<?xml version="1.0" standalone="yes"?>
<OpenDRIVE>
  <header revMajor="1" revMinor="9" name="Test Map" version="1.0" date="2026-06-14T09:00:00" north="100.0" south="-100.0" east="200.0" west="-200.0"/>
  <road name="Road 1" length="50.0" id="1" junction="-1" rule="RHT">
    <planView>
      <geometry s="0.0" x="0.0" y="0.0" hdg="0.0" length="20.0">
        <line/>
      </geometry>
      <geometry s="20.0" x="20.0" y="0.0" hdg="0.0" length="30.0">
        <arc curvature="0.1"/>
      </geometry>
    </planView>
    <lanes>
      <laneSection s="0.0">
        <center>
          <lane id="0"/>
        </center>
      </laneSection>
    </lanes>
  </road>
</OpenDRIVE>)";

  auto ast = strada::parser::ParseString(kXml);
  auto ref_line = ReferenceLine::Build(ast);

  EXPECT_EQ(ref_line.TotalSegmentsCount(), 2);

  // Test 1: Project point onto line segment (seg_idx = 0)
  // Point: (10.0, 5.0). Closest point on line should be (10.0, 0.0), s = 10.0
  double s_proj_line = ref_line.Project(0, 10.0, 5.0);
  EXPECT_NEAR(s_proj_line, 10.0, 1e-6);

  // Test 2: Project point onto arc segment (seg_idx = 1)
  // Arc starts at s = 20.0, curvature = 0.1 (radius = 10.0, center at (20.0, 10.0))
  // Query point at center of the arc's circle (20.0, 10.0) -> any point is equidistant,
  // but query at (20.0, 15.0) which is on the radial line at hdg = 0.0 (theta = -pi/2 from center).
  // The closest point on arc is (20.0, 0.0) at start s = 20.0.
  double s_proj_arc = ref_line.Project(1, 20.0, 5.0);
  EXPECT_NEAR(s_proj_arc, 20.0, 1e-6);

  // Query at (20.0 + 10.0*sin(0.5), 10.0 - 10.0*cos(0.5)) rotated along arc.
  // The point on reference line at s = 25.0 (ds = 5.0, theta = curvature*ds = 0.5) is:
  // x = 20.0 + 10.0 * sin(0.5)
  // y = 10.0 - 10.0 * cos(0.5)
  // If we query at (x + 2.0*sin(0.5), y - 2.0*cos(0.5)) [2.0 meters offset along normal],
  // the closest point on the reference line must be exactly s = 25.0.
  double ds = 5.0;
  double theta = 0.1 * ds;
  double rx = 20.0 + 10.0 * std::sin(theta);
  double ry = 10.0 - 10.0 * std::cos(theta);

  double qx = rx + 2.0 * std::sin(theta);
  double qy = ry - 2.0 * std::cos(theta);

  double s_proj_arc_offset = ref_line.Project(1, qx, qy);
  EXPECT_NEAR(s_proj_arc_offset, 25.0, 1e-6);
}

TEST(ReferenceLineTest, ComputeSegmentAabb) {
  // Arrange
  const std::string kXml = R"(<?xml version="1.0" standalone="yes"?>
<OpenDRIVE>
  <header revMajor="1" revMinor="9" name="Test Map" version="1.0" date="2026-06-14T09:00:00" north="100.0" south="-100.0" east="200.0" west="-200.0"/>
  <road name="Road 1" length="50.0" id="1" junction="-1" rule="RHT">
    <planView>
      <geometry s="0.0" x="0.0" y="0.0" hdg="0.0" length="20.0">
        <line/>
      </geometry>
    </planView>
    <lanes>
      <laneSection s="0.0">
        <center>
          <lane id="0"/>
        </center>
      </laneSection>
    </lanes>
  </road>
</OpenDRIVE>)";

  auto ast = strada::parser::ParseString(kXml);
  auto ref_line = ReferenceLine::Build(ast);

  // Act
  auto aabb = ref_line.ComputeSegmentAabb(0, 0.5);

  // Assert
  EXPECT_NEAR(aabb.min_x, -0.5, 1e-9);
  EXPECT_NEAR(aabb.min_y, -0.5, 1e-9);
  EXPECT_NEAR(aabb.max_x, 20.5, 1e-9);
  EXPECT_NEAR(aabb.max_y, 0.5, 1e-9);
}

TEST(ReferenceLineTest, FindSegmentIndexAndCoherence) {
  // Arrange
  const std::string kXml = R"(<?xml version="1.0" standalone="yes"?>
<OpenDRIVE>
  <header revMajor="1" revMinor="9" name="Test Map" version="1.0" date="2026-06-14T09:00:00" north="100.0" south="-100.0" east="200.0" west="-200.0"/>
  <road name="Road 1" length="50.0" id="1" junction="-1" rule="RHT">
    <planView>
      <geometry s="0.0" x="0.0" y="0.0" hdg="0.0" length="20.0">
        <line/>
      </geometry>
      <geometry s="20.0" x="20.0" y="0.0" hdg="0.0" length="30.0">
        <line/>
      </geometry>
    </planView>
    <lanes>
      <laneSection s="0.0">
        <center>
          <lane id="0"/>
        </center>
      </laneSection>
    </lanes>
  </road>
</OpenDRIVE>)";

  auto ast = strada::parser::ParseString(kXml);
  auto ref_line = ReferenceLine::Build(ast);
  QueryContext ctx;

  // 1. Initial query: empty cache, should look up and find segment 0, and update cache
  uint32_t idx1 = ref_line.FindSegmentIndex(RoadId{0}, 10.0, ctx);
  EXPECT_EQ(idx1, 0);
  EXPECT_TRUE(ctx.last_road.has_value());
  EXPECT_EQ(*ctx.last_road, RoadId{0});
  EXPECT_TRUE(ctx.last_segment_idx.has_value());
  EXPECT_EQ(*ctx.last_segment_idx, 0);

  // 2. Query at s = 15.0: should hit the temporal cache fast path
  uint32_t idx2 = ref_line.FindSegmentIndex(RoadId{0}, 15.0, ctx);
  EXPECT_EQ(idx2, 0);

  // 3. Query at s = 25.0: cache miss (out of segment bounds), should update to segment 1
  uint32_t idx3 = ref_line.FindSegmentIndex(RoadId{0}, 25.0, ctx);
  EXPECT_EQ(idx3, 1);
  EXPECT_EQ(*ctx.last_segment_idx, 1);
}

}  // namespace strada::cpm
