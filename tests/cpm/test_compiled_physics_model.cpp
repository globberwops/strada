#include <gtest/gtest.h>

#include <cmath>
#include <filesystem>
#include <strada/cpm/compiled_physics_model.hpp>
#include <strada/parser/parser.hpp>

TEST(CompiledPhysicsModelTest, CompileAndQueryConstantCrossSectionSurface) {
  // Arrange
  const std::string kXml = R"(<?xml version="1.0" standalone="yes"?>
<OpenDRIVE>
  <header revMajor="1" revMinor="9" name="Test Map" version="1.0" date="2026-06-14T09:00:00" north="100.0" south="-100.0" east="200.0" west="-200.0"/>
  <road name="Road 1" length="100.0" id="1" junction="-1" rule="RHT">
    <planView>
      <geometry s="0.0" x="0.0" y="0.0" hdg="0.0" length="100.0">
        <line/>
      </geometry>
    </planView>
    <elevationProfile/>
    <lateralProfile>
      <crossSectionSurface>
        <tOffset>
          <coefficients s="0.0" a="-0.375"/>
        </tOffset>
        <surfaceStrips>
          <strip id="1">
            <constant>
              <coefficients s="0.0" a="0.45"/>
            </constant>
          </strip>
        </surfaceStrips>
      </crossSectionSurface>
    </lateralProfile>
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
  auto cpm_model = strada::cpm::BuildCompiledPhysicsModel(ast);

  // Assert basic inspection
  EXPECT_EQ(cpm_model.RoadCount(), 1);
  auto road_id_opt = cpm_model.RoadIdFromString("1");
  ASSERT_TRUE(road_id_opt.has_value());
  if (!road_id_opt.has_value()) {
    return;
  }
  auto road_id = *road_id_opt;
  EXPECT_EQ(cpm_model.OriginalRoadId(road_id), "1");

  // Query pose on the constant surface
  strada::cpm::RoadPose pose;
  pose.s = 10.0;
  pose.t = 2.0;
  pose.h = 0.0;
  pose.heading = 0.0;
  pose.pitch = 0.0;
  pose.roll = 0.0;
  pose.road = road_id;

  strada::cpm::QueryContext ctx;
  auto inertial = cpm_model.RoadToInertial(pose, ctx);

  // Assert matching coordinates and height offset
  EXPECT_DOUBLE_EQ(inertial.x, 10.0);
  EXPECT_DOUBLE_EQ(inertial.y, 2.0);
  EXPECT_DOUBLE_EQ(inertial.z, 0.45);  // Height offset evaluated from constant strip
  EXPECT_DOUBLE_EQ(inertial.heading, 0.0);
  EXPECT_DOUBLE_EQ(inertial.pitch, 0.0);
  EXPECT_DOUBLE_EQ(inertial.roll, 0.0);  // Natural roll is 0.0 for crossSectionSurface
}

TEST(CompiledPhysicsModelTest, QueryMultiStripCrossSectionSurface) {
  // Arrange
  std::filesystem::path data_dir = STRADA_TEST_DATA_DIR;
  std::filesystem::path file_path = data_dir / "cross_section_surface_road_1.xodr";
  auto ast = strada::parser::ParseFile(file_path);

  // Act
  auto cpm_model = strada::cpm::BuildCompiledPhysicsModel(ast);
  auto road_id_opt = cpm_model.RoadIdFromString("1");
  ASSERT_TRUE(road_id_opt.has_value());
  if (!road_id_opt.has_value()) {
    return;
  }
  auto road_id = *road_id_opt;

  strada::cpm::QueryContext ctx;

  // Query 1: Left side, inside strip 2
  // s = 0.0, t = 2.0
  // t_offset = -0.375 -> t_surf = 2.375
  // Left side: t_surf >= 0.0. t_target = 2.375
  // strip 1 (id=1): width = 1.5 -> t_accum = 1.5
  // strip 2 (id=2): width = inf -> matches, t_strip = 2.375 - 1.5 = 0.875, dt = 0.875
  // strip 2 height profile: linear a = -0.02 -> height = -0.02 * 0.875 = -0.0175
  {
    strada::cpm::RoadPose pose;
    pose.s = 0.0;
    pose.t = 2.0;
    pose.h = 0.0;
    pose.heading = 0.0;
    pose.pitch = 0.0;
    pose.roll = 0.0;
    pose.road = road_id;

    auto inertial = cpm_model.RoadToInertial(pose, ctx);
    EXPECT_NEAR(inertial.z, -0.0175, 1e-9);
  }

  // Query 2: Right side, inside strip -1
  // s = 0.0, t = -2.0
  // t_offset = -0.375 -> t_surf = -1.625
  // Right side: t_surf < 0.0. t_target = 1.625
  // strip -1 (id=-1): width = inf -> matches, t_strip = 1.625, dt = -1.625 (negative)
  // strip -1 height profile: linear a = 0.02 -> height = 0.02 * -1.625 = -0.0325
  {
    strada::cpm::RoadPose pose;
    pose.s = 0.0;
    pose.t = -2.0;
    pose.h = 0.0;
    pose.heading = 0.0;
    pose.pitch = 0.0;
    pose.roll = 0.0;
    pose.road = road_id;

    auto inertial = cpm_model.RoadToInertial(pose, ctx);
    EXPECT_NEAR(inertial.z, -0.0325, 1e-9);
  }

  // Query 3: Left side at s = 35.0 (where strip 1 width evaluated to 0.0)
  // s = 35.0, t = 2.0
  // t_offset = -0.375 -> t_surf = 2.375, t_target = 2.375
  // strip 1 (id=1): width at s=35.0: 1.5 - 0.09*(5^2) + 0.006*(5^3) = 0.0 -> t_accum = 0.0
  // strip 2 (id=2): matches, t_strip = 2.375, dt = 2.375
  // height = -0.02 * 2.375 = -0.0475
  {
    strada::cpm::RoadPose pose;
    pose.s = 35.0;
    pose.t = 2.0;
    pose.h = 0.0;
    pose.heading = 0.0;
    pose.pitch = 0.0;
    pose.roll = 0.0;
    pose.road = road_id;

    auto inertial = cpm_model.RoadToInertial(pose, ctx);
    EXPECT_NEAR(inertial.z, -0.0475, 1e-9);
  }
}

TEST(CompiledPhysicsModelTest, QueryRelativeModeCrossSectionSurface) {
  // Arrange
  const std::string kXml = R"(<?xml version="1.0" standalone="yes"?>
<OpenDRIVE>
  <header revMajor="1" revMinor="9" name="Test Map" version="1.0" date="2026-06-14T09:00:00" north="100.0" south="-100.0" east="200.0" west="-200.0"/>
  <road name="Road 1" length="100.0" id="1" junction="-1" rule="RHT">
    <planView>
      <geometry s="0.0" x="0.0" y="0.0" hdg="0.0" length="100.0">
        <line/>
      </geometry>
    </planView>
    <elevationProfile/>
    <lateralProfile>
      <crossSectionSurface>
        <tOffset>
          <coefficients s="0.0" a="0.0"/>
        </tOffset>
        <surfaceStrips>
          <strip id="1">
            <width>
              <coefficients s="0.0" a="2.0"/>
            </width>
            <constant>
              <coefficients s="0.0" a="0.5"/>
            </constant>
            <linear>
              <coefficients s="0.0" a="0.1"/>
            </linear>
          </strip>
          <strip id="2" mode="relative">
            <width>
              <coefficients s="0.0" a="3.0"/>
            </width>
            <constant>
              <coefficients s="0.0" a="0.2"/>
            </constant>
            <linear>
              <coefficients s="0.0" a="0.05"/>
            </linear>
          </strip>
        </surfaceStrips>
      </crossSectionSurface>
    </lateralProfile>
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
  auto cpm_model = strada::cpm::BuildCompiledPhysicsModel(ast);
  auto road_id_opt = cpm_model.RoadIdFromString("1");
  ASSERT_TRUE(road_id_opt.has_value());
  if (!road_id_opt.has_value()) {
    return;
  }
  auto road_id = *road_id_opt;

  strada::cpm::QueryContext ctx;

  // Query: Left side, inside strip 2 at s = 0.0, t = 3.5
  // t_offset = 0.0 -> t_surf = 3.5. t_target = 3.5
  // strip 1 (id=1): width = 2.0 -> t_accum = 2.0
  // strip 2 (id=2): matches, t_strip = 1.5, dt = 1.5
  // Outer strip 2 height: 0.2 + 0.05 * 1.5 = 0.275
  // Inner strip 1 boundary height (at dt = 2.0): 0.5 + 0.1 * 2.0 = 0.700
  // Accumulated height = 0.700 + 0.275 = 0.975
  {
    strada::cpm::RoadPose pose;
    pose.s = 0.0;
    pose.t = 3.5;
    pose.h = 0.0;
    pose.heading = 0.0;
    pose.pitch = 0.0;
    pose.roll = 0.0;
    pose.road = road_id;

    auto inertial = cpm_model.RoadToInertial(pose, ctx);
    EXPECT_NEAR(inertial.z, 0.975, 1e-9);
  }
}

TEST(CompiledPhysicsModelTest, QueryLineAndArcReferenceLine) {
  // Arrange
  std::filesystem::path data_dir = STRADA_TEST_DATA_DIR;
  std::filesystem::path file_path = data_dir / "geometry.xodr";
  auto ast = strada::parser::ParseFile(file_path);

  // Act
  auto cpm_model = strada::cpm::BuildCompiledPhysicsModel(ast);
  auto road_id_opt = cpm_model.RoadIdFromString("1");
  ASSERT_TRUE(road_id_opt.has_value());
  if (!road_id_opt.has_value()) {
    return;
  }
  auto road_id = *road_id_opt;

  // Test inspection APIs
  EXPECT_EQ(cpm_model.RoadCount(), 1);
  EXPECT_EQ(cpm_model.OriginalRoadId(road_id), "1");
  EXPECT_DOUBLE_EQ(cpm_model.RoadLength(road_id), 100.0);

  strada::cpm::QueryContext ctx;

  // Query 1: On the Line segment (s = 5.0, t = 2.0, h = 0.5)
  // Reference point calculations:
  // hdg = 0.5, start_x = 10.0, start_y = 20.0
  // X = 10.0 + 5.0 * cos(0.5) - 2.0 * sin(0.5) = 13.429061732243458
  // Y = 20.0 + 5.0 * sin(0.5) + 2.0 * cos(0.5) = 24.15229281680176
  // Z = 0.5
  // Orientation: heading = 0.5, pitch = 0.0, roll = 0.0 (no offsets)
  {
    strada::cpm::RoadPose pose;
    pose.s = 5.0;
    pose.t = 2.0;
    pose.h = 0.5;
    pose.heading = 0.0;
    pose.pitch = 0.0;
    pose.roll = 0.0;
    pose.road = road_id;

    auto inertial = cpm_model.RoadToInertial(pose, ctx);
    EXPECT_NEAR(inertial.x, 13.429061732243458, 1e-9);
    EXPECT_NEAR(inertial.y, 24.15229281680176, 1e-9);
    EXPECT_NEAR(inertial.z, 0.5, 1e-9);
    EXPECT_NEAR(inertial.heading, 0.5, 1e-9);
    EXPECT_NEAR(inertial.pitch, 0.0, 1e-9);
    EXPECT_NEAR(inertial.roll, 0.0, 1e-9);
  }

  // Query 2: On the Arc segment (s = 30.0, t = -1.0, h = 0.2)
  // Segment start: s_start = 25.0, start_x = 35.0, start_y = 45.0, hdg = 0.7, curvature = 0.05
  // ds = 5.0, hdg_at_s = 0.7 + 0.05 * 5.0 = 0.95
  // X = 35.0 + 20.0 * (sin(0.95) - sin(0.7)) + sin(0.95) = 39.19737185582303
  // Y = 45.0 - 20.0 * (cos(0.95) - cos(0.7)) - cos(0.95) = 48.08149886694822
  // Z = 0.2
  // Orientation: heading = 0.95, pitch = 0.0, roll = 0.0
  {
    strada::cpm::RoadPose pose;
    pose.s = 30.0;
    pose.t = -1.0;
    pose.h = 0.2;
    pose.heading = 0.0;
    pose.pitch = 0.0;
    pose.roll = 0.0;
    pose.road = road_id;

    auto inertial = cpm_model.RoadToInertial(pose, ctx);
    EXPECT_NEAR(inertial.x, 39.19737185582303, 1e-9);
    EXPECT_NEAR(inertial.y, 48.08149886694822, 1e-9);
    EXPECT_NEAR(inertial.z, 0.2, 1e-9);
    EXPECT_NEAR(inertial.heading, 0.95, 1e-9);
    EXPECT_NEAR(inertial.pitch, 0.0, 1e-9);
    EXPECT_NEAR(inertial.roll, 0.0, 1e-9);
  }

  // Query 3: On the Line segment with orientation offsets
  // s = 5.0, t = 2.0, h = 0.5
  // pose.heading = 0.1, pose.pitch = 0.2, pose.roll = 0.3
  // Expected composed orientation: heading = 0.6, pitch = 0.2, roll = 0.3
  {
    strada::cpm::RoadPose pose;
    pose.s = 5.0;
    pose.t = 2.0;
    pose.h = 0.5;
    pose.heading = 0.1;
    pose.pitch = 0.2;
    pose.roll = 0.3;
    pose.road = road_id;

    auto inertial = cpm_model.RoadToInertial(pose, ctx);
    EXPECT_NEAR(inertial.heading, 0.6, 1e-9);
    EXPECT_NEAR(inertial.pitch, 0.2, 1e-9);
    EXPECT_NEAR(inertial.roll, 0.3, 1e-9);
  }
}

TEST(CompiledPhysicsModelTest, QuerySpiralReferenceLine) {
  // Arrange
  std::filesystem::path data_dir = STRADA_TEST_DATA_DIR;
  std::filesystem::path file_path = data_dir / "spiral_road.xodr";
  auto ast = strada::parser::ParseFile(file_path);

  // Act
  auto cpm_model = strada::cpm::BuildCompiledPhysicsModel(ast);
  auto road_id_opt = cpm_model.RoadIdFromString("1");
  ASSERT_TRUE(road_id_opt.has_value());
  if (!road_id_opt.has_value()) {
    return;
  }
  auto road_id = *road_id_opt;

  strada::cpm::QueryContext ctx;

  // Query 1: s = 20.0
  {
    strada::cpm::RoadPose pose;
    pose.s = 20.0;
    pose.t = 0.0;
    pose.h = 0.0;
    pose.heading = 0.0;
    pose.pitch = 0.0;
    pose.roll = 0.0;
    pose.road = road_id;

    auto inertial = cpm_model.RoadToInertial(pose, ctx);
    EXPECT_NEAR(inertial.x, 19.68236163732836, 1e-9);
    EXPECT_NEAR(inertial.y, 2.636345195025855, 1e-9);
    EXPECT_NEAR(inertial.z, 0.0, 1e-9);
    EXPECT_NEAR(inertial.heading, 0.4, 1e-9);
    EXPECT_NEAR(inertial.pitch, 0.0, 1e-9);
    EXPECT_NEAR(inertial.roll, 0.0, 1e-9);
  }

  // Query 2: s = 40.0
  {
    strada::cpm::RoadPose pose;
    pose.s = 40.0;
    pose.t = 0.0;
    pose.h = 0.0;
    pose.heading = 0.0;
    pose.pitch = 0.0;
    pose.roll = 0.0;
    pose.road = road_id;

    auto inertial = cpm_model.RoadToInertial(pose, ctx);
    EXPECT_NEAR(inertial.x, 30.904381740904647, 1e-9);
    EXPECT_NEAR(inertial.y, 17.7363194413844, 1e-9);
    EXPECT_NEAR(inertial.z, 0.0, 1e-9);
    EXPECT_NEAR(inertial.heading, 1.6, 1e-9);
    EXPECT_NEAR(inertial.pitch, 0.0, 1e-9);
    EXPECT_NEAR(inertial.roll, 0.0, 1e-9);
  }

  // Verify QueryContext fast-path engagement
  EXPECT_TRUE(ctx.last_road.has_value());
  if (ctx.last_road.has_value()) {
    EXPECT_EQ(*ctx.last_road, road_id);
  }
  EXPECT_TRUE(ctx.last_segment_idx.has_value());
  if (ctx.last_segment_idx.has_value()) {
    EXPECT_EQ(*ctx.last_segment_idx, 0U);
  }
}

TEST(CompiledPhysicsModelTest, QueryPoly3AndParamPoly3ReferenceLine) {
  // Arrange
  std::filesystem::path data_dir = STRADA_TEST_DATA_DIR;
  std::filesystem::path file_path = data_dir / "geometry.xodr";
  auto ast = strada::parser::ParseFile(file_path);

  // Act
  auto cpm_model = strada::cpm::BuildCompiledPhysicsModel(ast);
  auto road_id_opt = cpm_model.RoadIdFromString("1");
  ASSERT_TRUE(road_id_opt.has_value());
  if (!road_id_opt.has_value()) {
    return;
  }
  auto road_id = *road_id_opt;

  strada::cpm::QueryContext ctx;

  // 1. Verify ParamPoly3 evaluation (s = 85.0)
  // Coordinates calculated from:
  // p = 15.0
  // u(p) = 1.1 + 1.2*p + 1.3*p^2 + 1.4*p^3 = 5036.6
  // v(p) = 2.1 + 2.2*p + 2.3*p^2 + 2.4*p^3 = 8652.6
  // start x = 75.0, y = 85.0, hdg = 0.9
  // X = 75.0 + u*cos(0.9) - v*sin(0.9) = -3572.0136520507344
  // Y = 85.0 + u*sin(0.9) + v*cos(0.9) = 9408.846724488534
  // tangent_hdg = 0.9 + atan2(v', u') = 1.943310312623995
  {
    strada::cpm::RoadPose pose;
    pose.s = 85.0;
    pose.t = 0.0;
    pose.h = 0.0;
    pose.heading = 0.0;
    pose.pitch = 0.0;
    pose.roll = 0.0;
    pose.road = road_id;

    auto inertial = cpm_model.RoadToInertial(pose, ctx);
    EXPECT_NEAR(inertial.x, -3572.0136520507344, 1e-9);
    EXPECT_NEAR(inertial.y, 9408.846724488534, 1e-9);
    EXPECT_NEAR(inertial.heading, 1.943310312623995, 1e-9);
  }

  // 2. Query Poly3 at start (s = 45.0, which has ds_val = 0.0, u = 0.0)
  // u = 0.0, v = 1.0. Start x = 50.0, y = 60.0, hdg = 0.8
  // X = 50.0 + 0 * cos(0.8) - 1.0 * sin(0.8) = 50.0 - sin(0.8) = 49.28264390910048
  // Y = 60.0 + 0 * sin(0.8) + 1.0 * cos(0.8) = 60.0 + cos(0.8) = 60.696706709347165
  // hdg = 0.8 + atan2(2.0, 1.0) = 1.9071487177940904
  {
    strada::cpm::RoadPose pose;
    pose.s = 45.0;
    pose.t = 0.0;
    pose.h = 0.0;
    pose.heading = 0.0;
    pose.pitch = 0.0;
    pose.roll = 0.0;
    pose.road = road_id;

    auto inertial = cpm_model.RoadToInertial(pose, ctx);
    EXPECT_NEAR(inertial.x, 49.28264390910048, 1e-9);
    EXPECT_NEAR(inertial.y, 60.696706709347165, 1e-9);
    EXPECT_NEAR(inertial.heading, 1.9071487177940904, 1e-9);
  }

  // 3. Query Poly3 at middle (s = 57.5, which has ds_val = 12.5)
  // Expected coordinates from high-precision original Poly3 integration:
  // X = 41.180971657746085
  // Y = 70.171768309794714
  {
    strada::cpm::RoadPose pose;
    pose.s = 57.5;
    pose.t = 0.0;
    pose.h = 0.0;
    pose.heading = 0.0;
    pose.pitch = 0.0;
    pose.roll = 0.0;
    pose.road = road_id;

    auto inertial = cpm_model.RoadToInertial(pose, ctx);
    EXPECT_NEAR(inertial.x, 41.180971657746085, 1e-9);
    EXPECT_NEAR(inertial.y, 70.171768309794714, 1e-9);
    EXPECT_NEAR(inertial.heading, 2.466819223986507, 1e-9);
  }

  // 4. Query Poly3 at end (s = 70.0 - 1e-12, which has ds_val approx 25.0)
  // Expected coordinates from high-precision original Poly3 integration:
  // X = 32.490265485137456
  // Y = 79.156094908899561
  {
    strada::cpm::RoadPose pose;
    pose.s = 70.0 - 1e-12;
    pose.t = 0.0;
    pose.h = 0.0;
    pose.heading = 0.0;
    pose.pitch = 0.0;
    pose.roll = 0.0;
    pose.road = road_id;

    auto inertial = cpm_model.RoadToInertial(pose, ctx);
    EXPECT_NEAR(inertial.x, 32.490265485137456, 1e-9);
    EXPECT_NEAR(inertial.y, 79.156094908899561, 1e-9);
    EXPECT_NEAR(inertial.heading, 2.026619153539606, 1e-9);
  }
}

TEST(CompiledPhysicsModelTest, BuildStaticFactory) {
  // Arrange
  const std::string kXml = R"(<?xml version="1.0" standalone="yes"?>
<OpenDRIVE>
  <header revMajor="1" revMinor="9" name="Test Map" version="1.0" date="2026-06-14T09:00:00"/>
  <road name="Road 1" length="10.0" id="1" junction="-1">
    <planView>
      <geometry s="0.0" x="0.0" y="0.0" hdg="0.0" length="10.0">
        <line/>
      </geometry>
    </planView>
    <elevationProfile/>
    <lateralProfile/>
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
  auto model = strada::cpm::CompiledPhysicsModel::Build(ast);

  // Assert
  EXPECT_EQ(model.RoadCount(), 1);
}

TEST(CompiledPhysicsModelTest, LaneTransforms) {
  // Arrange
  std::filesystem::path data_dir = STRADA_TEST_DATA_DIR;
  std::filesystem::path file_path = data_dir / "lanes_and_profiles.xodr";
  auto ast = strada::parser::ParseFile(file_path);
  auto cpm_model = strada::cpm::BuildCompiledPhysicsModel(ast);

  auto lane0 = strada::cpm::LaneId{0};  // Original ID: -1
  auto lane2 = strada::cpm::LaneId{2};  // Original ID: 1

  strada::cpm::QueryContext ctx;

  // Act & Assert 1: Query left lane (lane2, original ID 1) at s = 10.0, t = 0.0, h = 0.0
  {
    strada::cpm::LanePose pose;
    pose.s = 10.0;
    pose.t = 0.0;
    pose.h = 0.0;
    pose.heading = 0.0;
    pose.pitch = 0.0;
    pose.roll = 0.0;
    pose.road = strada::cpm::RoadId{0};
    pose.lane = lane2;

    auto road_pose = cpm_model.LaneToRoad(pose, ctx);
    EXPECT_NEAR(road_pose.s, 10.0, 1e-9);
    EXPECT_NEAR(road_pose.t, 878.5, 1e-9);
    EXPECT_NEAR(road_pose.h, 0.05, 1e-9);
    EXPECT_EQ(road_pose.road, strada::cpm::RoadId{0});

    auto inertial_pose = cpm_model.LaneToInertial(pose, ctx);
    auto expected_inertial = cpm_model.RoadToInertial(road_pose, ctx);
    EXPECT_NEAR(inertial_pose.x, expected_inertial.x, 1e-9);
    EXPECT_NEAR(inertial_pose.y, expected_inertial.y, 1e-9);
    EXPECT_NEAR(inertial_pose.z, expected_inertial.z, 1e-9);
    EXPECT_NEAR(inertial_pose.heading, expected_inertial.heading, 1e-9);
    EXPECT_NEAR(inertial_pose.pitch, expected_inertial.pitch, 1e-9);
    EXPECT_NEAR(inertial_pose.roll, expected_inertial.roll, 1e-9);
  }

  // Act & Assert 2: Query right lane (lane0, original ID -1) at s = 10.0, t = -0.5, h = 0.0
  {
    strada::cpm::LanePose pose;
    pose.s = 10.0;
    pose.t = -0.5;
    pose.h = 0.0;
    pose.heading = 0.0;
    pose.pitch = 0.0;
    pose.roll = 0.0;
    pose.road = strada::cpm::RoadId{0};
    pose.lane = lane0;

    auto road_pose = cpm_model.LaneToRoad(pose, ctx);
    EXPECT_NEAR(road_pose.s, 10.0, 1e-9);
    EXPECT_NEAR(road_pose.t, 873.5, 1e-9);
    EXPECT_NEAR(road_pose.h, 0.0, 1e-9);
    EXPECT_EQ(road_pose.road, strada::cpm::RoadId{0});

    auto inertial_pose = cpm_model.LaneToInertial(pose, ctx);
    auto expected_inertial = cpm_model.RoadToInertial(road_pose, ctx);
    EXPECT_NEAR(inertial_pose.x, expected_inertial.x, 1e-9);
    EXPECT_NEAR(inertial_pose.y, expected_inertial.y, 1e-9);
    EXPECT_NEAR(inertial_pose.z, expected_inertial.z, 1e-9);
    EXPECT_NEAR(inertial_pose.heading, expected_inertial.heading, 1e-9);
    EXPECT_NEAR(inertial_pose.pitch, expected_inertial.pitch, 1e-9);
    EXPECT_NEAR(inertial_pose.roll, expected_inertial.roll, 1e-9);
  }

  // Act & Assert 3: Verify QueryContext caches road segment index
  {
    EXPECT_TRUE(ctx.last_road.has_value());
    EXPECT_EQ(*ctx.last_road, strada::cpm::RoadId{0});
    EXPECT_TRUE(ctx.last_segment_idx.has_value());
    EXPECT_EQ(*ctx.last_segment_idx, 0U);
  }
}

TEST(CompiledPhysicsModelTest, BoundingVolumeHierarchyConstructionAndLayout) {
  // Arrange
  std::filesystem::path data_dir = STRADA_TEST_DATA_DIR;
  std::filesystem::path file_path = data_dir / "geometry.xodr";
  auto ast = strada::parser::ParseFile(file_path);

  // Act
  auto cpm_model = strada::cpm::BuildCompiledPhysicsModel(ast);
  const auto& nodes = cpm_model.GetBoundingVolumeHierarchyNodes();
  const auto& primitives = cpm_model.GetBoundingVolumeHierarchyPrimitives();

  // Assert
  // Since geometry.xodr has 1 road with multiple plan-view geometry segments, we expect at least 1 bounding volume
  // hierarchy node
  ASSERT_FALSE(nodes.empty());
  ASSERT_FALSE(primitives.empty());

  // Root node should encompass all geometry
  const auto& root = nodes[0];
  EXPECT_LT(root.min_x, root.max_x);
  EXPECT_LT(root.min_y, root.max_y);

  // Traverse the bounding volume hierarchy and assert properties
  for (const auto& node : nodes) {
    bool is_leaf = (node.right & 0x80000000) != 0;
    if (is_leaf) {
      uint32_t start = node.left;
      uint32_t count = node.right & 0x7FFFFFFF;
      EXPECT_LT(start, primitives.size());
      EXPECT_LE(start + count, primitives.size());
      EXPECT_GT(count, 0U);
    } else {
      uint32_t left_child = node.left;
      uint32_t right_child = node.right & 0x7FFFFFFF;
      EXPECT_LT(left_child, nodes.size());
      EXPECT_LT(right_child, nodes.size());
      EXPECT_NE(left_child, 0U);
      EXPECT_NE(right_child, 0U);
    }
  }
}

TEST(CompiledPhysicsModelTest, InertialToRoadSnapping) {
  // Arrange
  std::filesystem::path data_dir = STRADA_TEST_DATA_DIR;
  std::filesystem::path file_path = data_dir / "geometry.xodr";
  auto ast = strada::parser::ParseFile(file_path);
  auto cpm_model = strada::cpm::BuildCompiledPhysicsModel(ast);
  auto road_id = *cpm_model.RoadIdFromString("1");

  strada::cpm::QueryContext ctx;

  // Act & Assert 1: Snap to Line segment
  // Expected pose: s=5.0, t=2.0, h=0.5
  {
    strada::cpm::InertialPose ip;
    ip.x = 13.429061732243458;
    ip.y = 24.15229281680176;
    ip.z = 0.5;
    ip.heading = 0.5;  // Natural tangent heading is 0.5
    ip.pitch = 0.0;
    ip.roll = 0.0;

    auto rp_opt = cpm_model.InertialToRoad(ip, ctx);
    ASSERT_TRUE(rp_opt.has_value());
    auto rp = *rp_opt;
    EXPECT_EQ(rp.road, road_id);
    EXPECT_NEAR(rp.s, 5.0, 1e-6);
    EXPECT_NEAR(rp.t, 2.0, 1e-6);
    EXPECT_NEAR(rp.h, 0.5, 1e-6);
  }

  // Act & Assert 2: Snap to Arc segment
  // Expected pose: s=30.0, t=-1.0, h=0.2
  {
    strada::cpm::InertialPose ip;
    ip.x = 39.19737185582303;
    ip.y = 48.08149886694822;
    ip.z = 0.2;
    ip.heading = 0.95;  // Natural tangent heading is 0.95
    ip.pitch = 0.0;
    ip.roll = 0.0;

    auto rp_opt = cpm_model.InertialToRoad(ip, ctx);
    ASSERT_TRUE(rp_opt.has_value());
    auto rp = *rp_opt;
    EXPECT_EQ(rp.road, road_id);
    EXPECT_NEAR(rp.s, 30.0, 1e-6);
    EXPECT_NEAR(rp.t, -1.0, 1e-6);
    EXPECT_NEAR(rp.h, 0.2, 1e-6);
  }

  // Act & Assert 3: Snap to ParamPoly3 segment
  // Expected pose: s=85.0, t=0.0, h=0.0
  {
    strada::cpm::InertialPose ip;
    ip.x = -3572.0136520507344;
    ip.y = 9408.846724488534;
    ip.z = 0.0;
    ip.heading = 1.943310312623995;
    ip.pitch = 0.0;
    ip.roll = 0.0;

    auto rp_opt = cpm_model.InertialToRoad(ip, ctx);
    ASSERT_TRUE(rp_opt.has_value());
    auto rp = *rp_opt;
    EXPECT_EQ(rp.road, road_id);
    EXPECT_NEAR(rp.s, 85.0, 1e-3);  // Numerical projection tolerance
    EXPECT_NEAR(rp.t, 0.0, 1e-3);
    EXPECT_NEAR(rp.h, 0.0, 1e-3);
  }
}

TEST(CompiledPhysicsModelTest, OrientationStripping) {
  // Arrange
  std::filesystem::path data_dir = STRADA_TEST_DATA_DIR;
  std::filesystem::path file_path = data_dir / "geometry.xodr";
  auto ast = strada::parser::ParseFile(file_path);
  auto cpm_model = strada::cpm::BuildCompiledPhysicsModel(ast);

  strada::cpm::QueryContext ctx;

  // Let's query a point on the Line segment (s=5.0, t=0.0, h=0.0)
  // Tangent heading is 0.5.
  // Natural pitch and roll are 0.0 because there's no elevation/superelevation.
  // If we pass an InertialPose with heading = 0.5, pitch = 0.0, roll = 0.0:
  {
    strada::cpm::InertialPose ip;
    ip.x = 10.0 + (5.0 * std::cos(0.5));
    ip.y = 20.0 + (5.0 * std::sin(0.5));
    ip.z = 0.0;
    ip.heading = 0.5;
    ip.pitch = 0.0;
    ip.roll = 0.0;

    auto rp_opt = cpm_model.InertialToRoad(ip, ctx);
    ASSERT_TRUE(rp_opt.has_value());
    auto rp = *rp_opt;
    EXPECT_NEAR(rp.heading, 0.0, 1e-9);
    EXPECT_NEAR(rp.pitch, 0.0, 1e-9);
    EXPECT_NEAR(rp.roll, 0.0, 1e-9);
  }

  // If we pass an InertialPose with heading = 0.6, pitch = 0.2, roll = 0.3
  // (which represents offset heading = 0.1, pitch = 0.2, roll = 0.3 relative to the road):
  {
    // The road rotation at s=5 is: R_road = R_z(0.5)
    // The query inertial rotation is: R_inertial = R_road * R_offset
    // Where R_offset = R_z(0.1) * R_y(0.2) * R_x(0.3)
    // We compose R_road and R_offset:
    strada::cpm::RoadPose rp_target;
    rp_target.s = 5.0;
    rp_target.t = 0.0;
    rp_target.h = 0.0;
    rp_target.heading = 0.1;
    rp_target.pitch = 0.2;
    rp_target.roll = 0.3;
    rp_target.road = strada::cpm::RoadId{0};

    auto ip = cpm_model.RoadToInertial(rp_target, ctx);

    // Now snap this ip back to road-relative coordinates:
    auto rp_opt = cpm_model.InertialToRoad(ip, ctx);
    ASSERT_TRUE(rp_opt.has_value());
    auto rp = *rp_opt;
    EXPECT_NEAR(rp.heading, 0.1, 1e-9);
    EXPECT_NEAR(rp.pitch, 0.2, 1e-9);
    EXPECT_NEAR(rp.roll, 0.3, 1e-9);
  }
}

TEST(CompiledPhysicsModelTest, BoundingVolumeHierarchyQueryContextFastPath) {
  // Arrange
  std::filesystem::path data_dir = STRADA_TEST_DATA_DIR;
  std::filesystem::path file_path = data_dir / "geometry.xodr";
  auto ast = strada::parser::ParseFile(file_path);
  auto cpm_model = strada::cpm::BuildCompiledPhysicsModel(ast);
  auto road_id = *cpm_model.RoadIdFromString("1");

  strada::cpm::QueryContext ctx;

  // 1. Cold query (inside the road) -> should succeed and populate context
  strada::cpm::InertialPose ip_first;
  ip_first.x = 10.0 + (5.0 * std::cos(0.5));
  ip_first.y = 20.0 + (5.0 * std::sin(0.5));
  ip_first.z = 0.0;
  ip_first.heading = 0.5;
  ip_first.pitch = 0.0;
  ip_first.roll = 0.0;

  auto rp_first = cpm_model.InertialToRoad(ip_first, ctx);
  ASSERT_TRUE(rp_first.has_value());
  EXPECT_TRUE(ctx.last_road.has_value());
  EXPECT_EQ(*ctx.last_road, road_id);

  // 2. Clear bounding volume hierarchy nodes in the model
  cpm_model.ClearBoundingVolumeHierarchyNodes();

  // 3. Warm query on a different point -> should still succeed via fast path because the cached road is used
  strada::cpm::InertialPose ip_second;
  ip_second.x = 10.0 + (6.0 * std::cos(0.5));
  ip_second.y = 20.0 + (6.0 * std::sin(0.5));
  ip_second.z = 0.0;
  ip_second.heading = 0.5;
  ip_second.pitch = 0.0;
  ip_second.roll = 0.0;

  auto rp_warm = cpm_model.InertialToRoad(ip_second, ctx);
  ASSERT_TRUE(rp_warm.has_value());
  EXPECT_NEAR(rp_warm->s, 6.0, 1e-6);

  // 4. Cold query (empty context) on the same second point -> should fail because bounding volume hierarchy is cleared
  strada::cpm::QueryContext ctx_cold;
  auto rp_cold = cpm_model.InertialToRoad(ip_second, ctx_cold);
  EXPECT_FALSE(rp_cold.has_value());
}

TEST(CompiledPhysicsModelTest, RoundTripInertialRoadInertial) {
  // Arrange
  std::filesystem::path data_dir = STRADA_TEST_DATA_DIR;
  std::filesystem::path file_path = data_dir / "geometry.xodr";
  auto ast = strada::parser::ParseFile(file_path);
  auto cpm_model = strada::cpm::BuildCompiledPhysicsModel(ast);

  strada::cpm::QueryContext ctx;

  // Let's test a point on the Line segment (s=5.0, t=1.0, h=0.2)
  {
    strada::cpm::RoadPose rp_orig;
    rp_orig.s = 5.0;
    rp_orig.t = 1.0;
    rp_orig.h = 0.2;
    rp_orig.heading = 0.1;
    rp_orig.pitch = -0.15;
    rp_orig.roll = 0.05;
    rp_orig.road = strada::cpm::RoadId{0};

    // Forward transformation
    auto ip = cpm_model.RoadToInertial(rp_orig, ctx);

    // Backward snapping
    auto rp_snap_opt = cpm_model.InertialToRoad(ip, ctx);
    ASSERT_TRUE(rp_snap_opt.has_value());
    auto rp_snap = *rp_snap_opt;

    // Convert snapped back to world
    auto ip_snap = cpm_model.RoadToInertial(rp_snap, ctx);

    // Verify round-trip matches the input world pose within 1e-9
    EXPECT_NEAR(ip_snap.x, ip.x, 1e-9);
    EXPECT_NEAR(ip_snap.y, ip.y, 1e-9);
    EXPECT_NEAR(ip_snap.z, ip.z, 1e-9);
    EXPECT_NEAR(ip_snap.heading, ip.heading, 1e-9);
    EXPECT_NEAR(ip_snap.pitch, ip.pitch, 1e-9);
    EXPECT_NEAR(ip_snap.roll, ip.roll, 1e-9);
  }

  // Let's test a point on the Arc segment (s=30.0, t=-1.5, h=0.3)
  {
    strada::cpm::RoadPose rp_orig;
    rp_orig.s = 30.0;
    rp_orig.t = -1.5;
    rp_orig.h = 0.3;
    rp_orig.heading = -0.2;
    rp_orig.pitch = 0.1;
    rp_orig.roll = -0.1;
    rp_orig.road = strada::cpm::RoadId{0};

    // Forward transformation
    auto ip = cpm_model.RoadToInertial(rp_orig, ctx);

    // Backward snapping
    auto rp_snap_opt = cpm_model.InertialToRoad(ip, ctx);
    ASSERT_TRUE(rp_snap_opt.has_value());
    auto rp_snap = *rp_snap_opt;

    // Convert snapped back to world
    auto ip_snap = cpm_model.RoadToInertial(rp_snap, ctx);

    // Verify round-trip matches the input world pose within 1e-9
    EXPECT_NEAR(ip_snap.x, ip.x, 1e-9);
    EXPECT_NEAR(ip_snap.y, ip.y, 1e-9);
    EXPECT_NEAR(ip_snap.z, ip.z, 1e-9);
    EXPECT_NEAR(ip_snap.heading, ip.heading, 1e-9);
    EXPECT_NEAR(ip_snap.pitch, ip.pitch, 1e-9);
    EXPECT_NEAR(ip_snap.roll, ip.roll, 1e-9);
  }
}

TEST(CompiledPhysicsModelTest, CrossPoseQueries) {
  // Arrange
  std::filesystem::path data_dir = STRADA_TEST_DATA_DIR;
  std::filesystem::path file_path = data_dir / "lanes_and_profiles.xodr";
  auto ast = strada::parser::ParseFile(file_path);
  auto cpm_model = strada::cpm::BuildCompiledPhysicsModel(ast);

  auto lane0 = strada::cpm::LaneId{0};  // Original ID: -1
  auto lane1 = strada::cpm::LaneId{1};  // Original ID: 0 (center)
  auto lane2 = strada::cpm::LaneId{2};  // Original ID: 1 (left)

  strada::cpm::QueryContext ctx;

  // Act & Assert 1: RoadToLane for left lane (lane2, original ID 1)
  // At s = 10.0, laneOffset is 876.5.
  // Left lane 2 starts from 0 to 4.0. Center of lane is at 2.0 (t = 878.5 in road frame).
  // So road pose with t = 878.5, h = 0.05 (with height offset 0.05) should snap to lane2 with t = 0.0, h = 0.0
  {
    strada::cpm::RoadPose rp;
    rp.s = 10.0;
    rp.t = 878.5;
    rp.h = 0.05;
    rp.heading = 0.15;
    rp.pitch = -0.2;
    rp.roll = 0.3;
    rp.road = strada::cpm::RoadId{0};

    auto lp_opt = cpm_model.RoadToLane(rp, ctx);
    ASSERT_TRUE(lp_opt.has_value());
    auto lp = *lp_opt;
    EXPECT_EQ(lp.lane, lane2);
    EXPECT_NEAR(lp.s, 10.0, 1e-9);
    EXPECT_NEAR(lp.t, 0.0, 1e-9);
    EXPECT_NEAR(lp.h, 0.0, 1e-9);
    EXPECT_NEAR(lp.heading, 0.15, 1e-9);
    EXPECT_NEAR(lp.pitch, -0.2, 1e-9);
    EXPECT_NEAR(lp.roll, 0.3, 1e-9);
    EXPECT_EQ(lp.road, strada::cpm::RoadId{0});
  }

  // Act & Assert 2: RoadToLane for right lane (lane0, original ID -1)
  // At s = 10.0, right lane spans from -5.0 to 0.0 (relative to offset).
  // With laneOffset = 876.5, the lane spans from 871.5 to 876.5.
  // Target center of lane is at -2.5 (t = 874.0 in road frame).
  // If we query t = 873.0, it is at relative offset -3.5. Since center is at -2.5, t_lane = -1.0.
  {
    strada::cpm::RoadPose rp;
    rp.s = 10.0;
    rp.t = 873.0;
    rp.h = 0.0;
    rp.heading = -0.1;
    rp.pitch = 0.25;
    rp.roll = -0.3;
    rp.road = strada::cpm::RoadId{0};

    auto lp_opt = cpm_model.RoadToLane(rp, ctx);
    ASSERT_TRUE(lp_opt.has_value());
    auto lp = *lp_opt;
    EXPECT_EQ(lp.lane, lane0);
    EXPECT_NEAR(lp.s, 10.0, 1e-9);
    EXPECT_NEAR(lp.t, -1.0, 1e-9);
    EXPECT_NEAR(lp.h, 0.0, 1e-9);
    EXPECT_NEAR(lp.heading, -0.1, 1e-9);
    EXPECT_NEAR(lp.pitch, 0.25, 1e-9);
    EXPECT_NEAR(lp.roll, -0.3, 1e-9);
  }

  // Act & Assert 3: RoadToLane for center lane/marking (t_relative = 0.0, i.e. road t = 876.5)
  // Since center lane has width 0, it shouldn't contain any point
  {
    strada::cpm::RoadPose rp;
    rp.s = 10.0;
    rp.t = 876.5;
    rp.h = 0.0;
    rp.road = strada::cpm::RoadId{0};

    auto lp_opt = cpm_model.RoadToLane(rp, ctx);
    EXPECT_FALSE(lp_opt.has_value());
  }

  // Act & Assert 4: RoadToLane for a point way outside the road lanes (e.g. t = 900.0)
  {
    strada::cpm::RoadPose rp;
    rp.s = 10.0;
    rp.t = 900.0;
    rp.h = 0.0;
    rp.road = strada::cpm::RoadId{0};

    auto lp_opt = cpm_model.RoadToLane(rp, ctx);
    EXPECT_FALSE(lp_opt.has_value());
  }
}

TEST(CompiledPhysicsModelTest, RoundTripLaneInertialLane) {
  // Arrange
  std::filesystem::path data_dir = STRADA_TEST_DATA_DIR;
  std::filesystem::path file_path = data_dir / "lanes_flat.xodr";
  auto ast = strada::parser::ParseFile(file_path);
  auto cpm_model = strada::cpm::BuildCompiledPhysicsModel(ast);

  auto lane2 = strada::cpm::LaneId{2};  // Original ID: 1

  strada::cpm::QueryContext ctx;

  // Act & Assert: LaneToInertial -> InertialToLane round-trip
  {
    strada::cpm::LanePose lp_orig;
    lp_orig.s = 10.0;
    lp_orig.t = 0.5;
    lp_orig.h = 0.1;
    lp_orig.heading = 0.1;
    lp_orig.pitch = -0.1;
    lp_orig.roll = 0.2;
    lp_orig.road = strada::cpm::RoadId{0};
    lp_orig.lane = lane2;

    auto ip = cpm_model.LaneToInertial(lp_orig, ctx);
    auto lp_snap_opt = cpm_model.InertialToLane(ip, ctx);

    ASSERT_TRUE(lp_snap_opt.has_value());
    auto lp_snap = *lp_snap_opt;
    EXPECT_EQ(lp_snap.lane, lp_orig.lane);
    EXPECT_NEAR(lp_snap.s, lp_orig.s, 1e-9);
    EXPECT_NEAR(lp_snap.t, lp_orig.t, 1e-9);
    EXPECT_NEAR(lp_snap.h, lp_orig.h, 1e-9);
    EXPECT_NEAR(lp_snap.heading, lp_orig.heading, 1e-9);
    EXPECT_NEAR(lp_snap.pitch, lp_orig.pitch, 1e-9);
    EXPECT_NEAR(lp_snap.roll, lp_orig.roll, 1e-9);
  }
}

TEST(CompiledPhysicsModelTest, BivariateShapeProfile) {
  // Arrange
  std::filesystem::path data_dir = STRADA_TEST_DATA_DIR;
  // Note: bivariate_shape_road.xodr is located in tests/cpm/data/
  std::filesystem::path file_path =
      std::filesystem::path(STRADA_TEST_DATA_DIR) / ".." / "cpm" / "data" / "bivariate_shape_road.xodr";
  auto ast = strada::parser::ParseFile(file_path);
  auto cpm_model = strada::cpm::BuildCompiledPhysicsModel(ast);

  strada::cpm::QueryContext ctx;

  // Act & Assert 1: RoadToInertial forward transform at s = 5.0, t = 2.0, h = 0.0
  {
    strada::cpm::RoadPose rp;
    rp.s = 5.0;
    rp.t = 2.0;
    rp.h = 0.0;
    rp.heading = 0.0;
    rp.pitch = 0.0;
    rp.roll = 0.0;
    rp.road = strada::cpm::RoadId{0};

    auto ip = cpm_model.RoadToInertial(rp, ctx);

    // Hand-calculated values
    double roll_total = std::atan(0.15);
    double cos_roll = std::cos(roll_total);
    double sin_roll = std::sin(roll_total);
    double expected_y = (cos_roll * 2.0) - (sin_roll * 0.8);
    double expected_z = (sin_roll * 2.0) + (cos_roll * 0.8);

    EXPECT_NEAR(ip.x, 5.0, 1e-6);
    EXPECT_NEAR(ip.y, expected_y, 1e-6);
    EXPECT_NEAR(ip.z, expected_z, 1e-6);
    EXPECT_NEAR(ip.heading, 0.0, 1e-6);
    EXPECT_NEAR(ip.pitch, 0.0, 1e-6);
    EXPECT_NEAR(ip.roll, roll_total, 1e-6);

    // Act & Assert 2: InertialToRoad backward snap round-trip
    auto rp_snap_opt = cpm_model.InertialToRoad(ip, ctx);
    ASSERT_TRUE(rp_snap_opt.has_value());
    auto rp_snap = *rp_snap_opt;
    EXPECT_EQ(rp_snap.road, rp.road);
    EXPECT_NEAR(rp_snap.s, rp.s, 1e-9);
    EXPECT_NEAR(rp_snap.t, rp.t, 1e-9);
    EXPECT_NEAR(rp_snap.h, rp.h, 1e-9);
    EXPECT_NEAR(rp_snap.heading, rp.heading, 1e-9);
    EXPECT_NEAR(rp_snap.pitch, rp.pitch, 1e-9);
    EXPECT_NEAR(rp_snap.roll, rp.roll, 1e-9);
  }
}
