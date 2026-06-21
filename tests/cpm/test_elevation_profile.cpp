// SPDX-License-Identifier: BSL-1.0

#include <gtest/gtest.h>

#include <cmath>
#include <strada/ast/abstract_syntax_tree.hpp>
#include <strada/cpm/elevation_profile.hpp>
#include <strada/cpm/ids.hpp>
#include <strada/parser/parser.hpp>

namespace strada::cpm {

TEST(ElevationProfileTest, CompileAndEvaluateElevationAndSuperelevation) {
  // Arrange: Simple road with linear elevation profile and superelevation profile.
  const std::string kXml = R"(<?xml version="1.0" standalone="yes"?>
<OpenDRIVE>
  <header revMajor="1" revMinor="9" name="Test Map" version="1.0" date="2026-06-14T09:00:00" north="100.0" south="-100.0" east="200.0" west="-200.0"/>
  <road name="Road 1" length="100.0" id="1" junction="-1" rule="RHT">
    <planView>
      <geometry s="0.0" x="0.0" y="0.0" hdg="0.0" length="100.0">
        <line/>
      </geometry>
    </planView>
    <elevationProfile>
      <elevation s="0.0" a="1.0" b="0.1" c="0.0" d="0.0"/>
    </elevationProfile>
    <lateralProfile>
      <superelevation s="0.0" a="0.02" b="0.0" c="0.0" d="0.0"/>
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

  auto ast = parser::ParseString(kXml);

  // Act
  auto profile = ElevationProfile::Build(ast);
  RoadId road{0};

  // Assert at s = 10.0, t = 0.0
  auto vp = profile.Evaluate(road, 10.0, 0.0);
  EXPECT_NEAR(vp.elevation, 2.0, 1e-9);         // 1.0 + 0.1 * 10
  EXPECT_NEAR(vp.pitch, std::atan(0.1), 1e-9);  // derivative is 0.1
  EXPECT_NEAR(vp.natural_roll, 0.02, 1e-9);
  EXPECT_NEAR(vp.roll_total, 0.02, 1e-9);
  EXPECT_NEAR(vp.shape_height, 0.0, 1e-9);
}

TEST(ElevationProfileTest, SuperelevationIsZeroIfCrossSectionSurfacePresent) {
  // Arrange: Road has superelevation AND cross-section surface. Superelevation must be ignored.
  const std::string kXml = R"(<?xml version="1.0" standalone="yes"?>
<OpenDRIVE>
  <header revMajor="1" revMinor="9" name="Test Map" version="1.0" date="2026-06-14T09:00:00"/>
  <road name="Road 1" length="100.0" id="1" junction="-1">
    <planView>
      <geometry s="0.0" x="0.0" y="0.0" hdg="0.0" length="100.0">
        <line/>
      </geometry>
    </planView>
    <elevationProfile/>
    <lateralProfile>
      <superelevation s="0.0" a="0.05" b="0.0" c="0.0" d="0.0"/>
      <crossSectionSurface>
        <tOffset>
          <coefficients s="0.0" a="0.0"/>
        </tOffset>
        <surfaceStrips>
          <strip id="1">
            <constant>
              <coefficients s="0.0" a="0.1"/>
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

  auto ast = parser::ParseString(kXml);

  // Act
  auto profile = ElevationProfile::Build(ast);
  RoadId road{0};

  // Assert
  auto vp = profile.Evaluate(road, 10.0, 0.0);
  EXPECT_NEAR(vp.natural_roll, 0.0, 1e-9);
  EXPECT_NEAR(vp.roll_total, 0.0, 1e-9);
}

TEST(ElevationProfileTest, ShapeProfileEvaluation) {
  // Arrange: Simple road with shape profile
  const std::string kXml = R"(<?xml version="1.0" standalone="yes"?>
<OpenDRIVE>
  <header revMajor="1" revMinor="9" name="Test Map" version="1.0" date="2026-06-14T09:00:00"/>
  <road name="Road 1" length="100.0" id="1" junction="-1">
    <planView>
      <geometry s="0.0" x="0.0" y="0.0" hdg="0.0" length="100.0">
        <line/>
      </geometry>
    </planView>
    <elevationProfile/>
    <lateralProfile>
      <shape s="0.0" t="0.0" a="0.5" b="0.1" c="0.0" d="0.0"/>
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

  auto ast = parser::ParseString(kXml);

  // Act
  auto profile = ElevationProfile::Build(ast);
  RoadId road{0};

  // Assert
  EXPECT_NEAR(profile.EvaluateShapeHeight(road, 10.0, 2.0), 0.7, 1e-9);     // 0.5 + 0.1 * (2 - 0)
  EXPECT_NEAR(profile.EvaluateShapeTGradient(road, 10.0, 2.0), 0.1, 1e-9);  // derivative is 0.1

  auto vp = profile.Evaluate(road, 10.0, 2.0);
  EXPECT_NEAR(vp.shape_height, 0.7, 1e-9);
  EXPECT_NEAR(vp.roll_total, std::atan(0.1), 1e-9);
}

}  // namespace strada::cpm
