#include <gtest/gtest.h>

#include <cmath>
#include <strada/cpm/coordinate.hpp>
#include <strada/cpm/elevation_profile.hpp>
#include <strada/cpm/lane_network.hpp>
#include <strada/cpm/reference_line.hpp>
#include <strada/parser/parser.hpp>

#include "../../src/cpm/road_projector.hpp"

namespace strada::cpm {

using namespace std::string_literals;

TEST(RoadProjectorTest, ProjectPointExactlyOnStraightRoad) {
  // Arrange
  const auto xml = R"(<?xml version="1.0" standalone="yes"?>
<OpenDRIVE>
  <header revMajor="1" revMinor="9" name="Test Map" version="1.0" date="2026-06-14T09:00:00" north="100.0" south="-100.0" east="200.0" west="-200.0"/>
  <road name="Road 1" length="100.0" id="1" junction="-1" rule="RHT">
    <planView>
      <geometry s="0.0" x="10.0" y="20.0" hdg="0.0" length="100.0">
        <line/>
      </geometry>
    </planView>
    <elevationProfile/>
    <lanes>
      <laneSection s="0.0">
        <center>
          <lane id="0" type="none"/>
        </center>
        <right>
          <lane id="-1" type="driving">
            <width s="0.0" a="3.0"/>
          </lane>
        </right>
      </laneSection>
    </lanes>
  </road>
</OpenDRIVE>)"s;

  const auto ast = parser::ParseString(xml);
  const ReferenceLine ref_line(ast);
  const ElevationProfile elevation_profile(ast);
  const LaneNetwork lane_network(ast);

  const auto projector = RoadProjector{ref_line, elevation_profile, lane_network};

  const auto pose = InertialPose{.x = 50.0, .y = 20.0, .z = 0.0, .heading = 0.0, .pitch = 0.0, .roll = 0.0};
  QueryContext ctx;

  // Act
  const auto projected_pose_opt = projector.Project(RoadId{0}, pose, ctx);

  // Assert
  ASSERT_TRUE(projected_pose_opt.has_value());
  EXPECT_NEAR(projected_pose_opt->s, 40.0, 1e-9);  // start x is 10.0, so s is 50.0 - 10.0 = 40.0
  EXPECT_NEAR(projected_pose_opt->t, 0.0, 1e-9);
  EXPECT_NEAR(projected_pose_opt->h, 0.0, 1e-9);
  EXPECT_NEAR(projected_pose_opt->heading, 0.0, 1e-9);
  EXPECT_EQ(projected_pose_opt->road, RoadId{0});
}

TEST(RoadProjectorTest, ProjectPointWithLateralOffsetWithinLanes) {
  // Arrange
  const auto xml = R"(<?xml version="1.0" standalone="yes"?>
<OpenDRIVE>
  <header revMajor="1" revMinor="9" name="Test Map" version="1.0" date="2026-06-14T09:00:00" north="100.0" south="-100.0" east="200.0" west="-200.0"/>
  <road name="Road 1" length="100.0" id="1" junction="-1" rule="RHT">
    <planView>
      <geometry s="0.0" x="0.0" y="0.0" hdg="0.0" length="100.0">
        <line/>
      </geometry>
    </planView>
    <elevationProfile/>
    <lanes>
      <laneSection s="0.0">
        <center>
          <lane id="0" type="none"/>
        </center>
        <right>
          <lane id="-1" type="driving">
            <width s="0.0" a="3.0"/>
          </lane>
        </right>
      </laneSection>
    </lanes>
  </road>
</OpenDRIVE>)"s;

  const auto ast = parser::ParseString(xml);
  const ReferenceLine ref_line(ast);
  const ElevationProfile elevation_profile(ast);
  const LaneNetwork lane_network(ast);

  const auto projector = RoadProjector{ref_line, elevation_profile, lane_network};

  // Road goes along x axis. Point is at x = 30.0, y = -2.0 (right side, within lane width limit of 3.0)
  const auto pose = InertialPose{.x = 30.0, .y = -2.0, .z = 0.0, .heading = 0.0, .pitch = 0.0, .roll = 0.0};
  QueryContext ctx;

  // Act
  const auto projected_pose_opt = projector.Project(RoadId{0}, pose, ctx);

  // Assert
  ASSERT_TRUE(projected_pose_opt.has_value());
  EXPECT_NEAR(projected_pose_opt->s, 30.0, 1e-9);
  EXPECT_NEAR(projected_pose_opt->t, -2.0, 1e-9);
  EXPECT_NEAR(projected_pose_opt->h, 0.0, 1e-9);
}

TEST(RoadProjectorTest, ProjectPointOutsideSnappingToleranceReturnsNullopt) {
  // Arrange
  const auto xml = R"(<?xml version="1.0" standalone="yes"?>
<OpenDRIVE>
  <header revMajor="1" revMinor="9" name="Test Map" version="1.0" date="2026-06-14T09:00:00" north="100.0" south="-100.0" east="200.0" west="-200.0"/>
  <road name="Road 1" length="100.0" id="1" junction="-1" rule="RHT">
    <planView>
      <geometry s="0.0" x="0.0" y="0.0" hdg="0.0" length="100.0">
        <line/>
      </geometry>
    </planView>
    <elevationProfile/>
    <lanes>
      <laneSection s="0.0">
        <center>
          <lane id="0" type="none"/>
        </center>
        <right>
          <lane id="-1" type="driving">
            <width s="0.0" a="3.0"/>
          </lane>
        </right>
      </laneSection>
    </lanes>
  </road>
</OpenDRIVE>)"s;

  const auto ast = parser::ParseString(xml);
  const ReferenceLine ref_line(ast);
  const ElevationProfile elevation_profile(ast);
  const LaneNetwork lane_network(ast);

  const auto projector = RoadProjector{ref_line, elevation_profile, lane_network};

  // Snapping tolerance is 5.0. Since lane width is 3.0, limit is -3.0.
  // A point at y = -9.0 is 6.0 units away from the lane limit, which exceeds the 5.0 snapping tolerance.
  const auto pose = InertialPose{.x = 30.0, .y = -9.0, .z = 0.0, .heading = 0.0, .pitch = 0.0, .roll = 0.0};
  QueryContext ctx;

  // Act
  const auto projected_pose_opt = projector.Project(RoadId{0}, pose, ctx);

  // Assert
  EXPECT_FALSE(projected_pose_opt.has_value());
}

}  // namespace strada::cpm
