#include <gtest/gtest.h>

#include <filesystem>
#include <strada/cpm/compiled_physics_model.hpp>
#include <strada/parser/parser.hpp>

TEST(CompiledPhysicsModelTest, CompileAndQueryConstantCrossSectionSurface) {
  // Arrange
  const std::string xml = R"(<?xml version="1.0" standalone="yes"?>
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

  auto ast = strada::parser::ParseString(xml);

  // Act
  auto cpm_model = strada::cpm::BuildCompiledPhysicsModel(ast);

  // Assert basic inspection
  EXPECT_EQ(cpm_model.road_count(), 1);
  auto road_id_opt = cpm_model.road_id_from_string("1");
  ASSERT_TRUE(road_id_opt.has_value());
  EXPECT_EQ(cpm_model.original_road_id(*road_id_opt), "1");

  // Query pose on the constant surface
  strada::cpm::RoadPose pose;
  pose.s = 10.0;
  pose.t = 2.0;
  pose.h = 0.0;
  pose.heading = 0.0;
  pose.pitch = 0.0;
  pose.roll = 0.0;
  pose.road = *road_id_opt;

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
  auto road_id_opt = cpm_model.road_id_from_string("1");
  ASSERT_TRUE(road_id_opt.has_value());

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
    pose.road = *road_id_opt;

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
    pose.road = *road_id_opt;

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
    pose.road = *road_id_opt;

    auto inertial = cpm_model.RoadToInertial(pose, ctx);
    EXPECT_NEAR(inertial.z, -0.0475, 1e-9);
  }
}

TEST(CompiledPhysicsModelTest, QueryRelativeModeCrossSectionSurface) {
  // Arrange
  const std::string xml = R"(<?xml version="1.0" standalone="yes"?>
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

  auto ast = strada::parser::ParseString(xml);

  // Act
  auto cpm_model = strada::cpm::BuildCompiledPhysicsModel(ast);
  auto road_id_opt = cpm_model.road_id_from_string("1");
  ASSERT_TRUE(road_id_opt.has_value());

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
    pose.road = *road_id_opt;

    auto inertial = cpm_model.RoadToInertial(pose, ctx);
    EXPECT_NEAR(inertial.z, 0.975, 1e-9);
  }
}
