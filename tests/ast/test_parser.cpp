#include <gtest/gtest.h>

#include <fstream>
#include <strada/parser/parser.hpp>

TEST(ParserTest, ParseMinimalHeaderFromString) {
  // Arrange
  std::string_view xml = R"(<?xml version="1.0" encoding="utf-8"?>
<OpenDRIVE>
    <header revMajor="1" revMinor="9" name="Test Map" version="1.0" date="2026-06-14T09:00:00" north="100.0" south="-100.0" east="200.0" west="-200.0" vendor="Strada Vendor">
        <geoReference><![CDATA[+proj=utm +zone=32 +datum=WGS84]]></geoReference>
    </header>
</OpenDRIVE>
)";

  // Act
  auto opendrive = strada::parser::ParseString(xml);

  // Assert
  const auto& header = opendrive.header_;
  EXPECT_EQ(header.rev_major_, 1);
  EXPECT_EQ(header.rev_minor_, 9);
  EXPECT_EQ(header.name_, "Test Map");
  EXPECT_EQ(header.version_, "1.0");
  EXPECT_EQ(header.date_, "2026-06-14T09:00:00");
  EXPECT_DOUBLE_EQ(header.north_, 100.0);
  EXPECT_DOUBLE_EQ(header.south_, -100.0);
  EXPECT_DOUBLE_EQ(header.east_, 200.0);
  EXPECT_DOUBLE_EQ(header.west_, -200.0);
  EXPECT_EQ(header.vendor_, "Strada Vendor");
  EXPECT_EQ(header.geo_reference_, "+proj=utm +zone=32 +datum=WGS84");
}

TEST(ParserTest, ParseMinimalHeaderFromFile) {
  // Arrange
  std::string_view xml = R"(<?xml version="1.0" encoding="utf-8"?>
<OpenDRIVE>
    <header revMajor="1" revMinor="9" name="Test Map" version="1.0" date="2026-06-14T09:00:00" north="100.0" south="-100.0" east="200.0" west="-200.0" vendor="Strada Vendor">
        <geoReference><![CDATA[+proj=utm +zone=32 +datum=WGS84]]></geoReference>
    </header>
</OpenDRIVE>
)";

  std::string temp_filename = "temp_test_map.xodr";
  {
    std::ofstream out(temp_filename);
    out << xml;
  }

  // Act
  auto opendrive = strada::parser::ParseFile(temp_filename);

  // Assert
  std::filesystem::remove(temp_filename);
  const auto& header = opendrive.header_;
  EXPECT_EQ(header.rev_major_, 1);
  EXPECT_EQ(header.rev_minor_, 9);
  EXPECT_EQ(header.name_, "Test Map");
  EXPECT_EQ(header.geo_reference_, "+proj=utm +zone=32 +datum=WGS84");
}

TEST(ParserTest, ParseRoadsFromString) {
  // Arrange
  std::string_view xml = R"(<?xml version="1.0" encoding="utf-8"?>
<OpenDRIVE>
    <header revMajor="1" revMinor="9" name="Test Map" version="1.0" date="2026-06-14T09:00:00" north="100.0" south="-100.0" east="200.0" west="-200.0" vendor="Strada Vendor">
        <geoReference><![CDATA[+proj=utm +zone=32 +datum=WGS84]]></geoReference>
    </header>
    <road id="1" length="10.0" junction="-1" rule="RHT" name="Road 1"/>
    <road id="2" length="25.5" junction="42" rule="LHT"/>
</OpenDRIVE>
)";

  // Act
  auto opendrive = strada::parser::ParseString(xml);

  // Assert
  const auto& roads = opendrive.roads_;
  ASSERT_EQ(roads.size(), 2);

  EXPECT_EQ(roads[0].id_, "1");
  EXPECT_DOUBLE_EQ(roads[0].length_, 10.0);
  EXPECT_EQ(roads[0].junction_, "-1");
  EXPECT_EQ(roads[0].rule_, "RHT");
  EXPECT_EQ(roads[0].name_, "Road 1");

  EXPECT_EQ(roads[1].id_, "2");
  EXPECT_DOUBLE_EQ(roads[1].length_, 25.5);
  EXPECT_EQ(roads[1].junction_, "42");
  EXPECT_EQ(roads[1].rule_, "LHT");
  EXPECT_EQ(roads[1].name_, "");
}

TEST(ParserTest, ParseGeometryFromPlanView) {
  // Arrange
  std::string_view xml = R"(<?xml version="1.0" encoding="utf-8"?>
<OpenDRIVE>
    <header revMajor="1" revMinor="9" name="Test Geometry Map" version="1.0" date="2026-06-14T09:00:00" north="100.0" south="-100.0" east="200.0" west="-200.0" vendor="Strada Vendor">
        <geoReference><![CDATA[+proj=utm +zone=32 +datum=WGS84]]></geoReference>
    </header>
    <road id="1" length="100.0" junction="-1">
        <planView>
            <geometry s="0.0" x="10.0" y="20.0" hdg="0.5" length="10.0">
                <line/>
            </geometry>
            <geometry s="10.0" x="20.0" y="30.0" hdg="0.6" length="15.0">
                <spiral curvStart="0.0" curvEnd="0.1"/>
            </geometry>
            <geometry s="25.0" x="35.0" y="45.0" hdg="0.7" length="20.0">
                <arc curvature="0.05"/>
            </geometry>
            <geometry s="45.0" x="50.0" y="60.0" hdg="0.8" length="25.0">
                <poly3 a="1.0" b="2.0" c="3.0" d="4.0"/>
            </geometry>
            <geometry s="70.0" x="75.0" y="85.0" hdg="0.9" length="30.0">
                <paramPoly3 aU="1.1" bU="1.2" cU="1.3" dU="1.4" aV="2.1" bV="2.2" cV="2.3" dV="2.4" pRange="arcLength"/>
            </geometry>
        </planView>
    </road>
</OpenDRIVE>
)";

  // Act
  auto opendrive = strada::parser::ParseString(xml);

  // Assert
  ASSERT_EQ(opendrive.roads_.size(), 1);
  const auto& road = opendrive.roads_[0];
  const auto& plan_view = road.plan_view_;
  ASSERT_EQ(plan_view.size(), 5);

  // 1. Line
  EXPECT_DOUBLE_EQ(plan_view[0].s_, 0.0);
  EXPECT_DOUBLE_EQ(plan_view[0].x_, 10.0);
  EXPECT_DOUBLE_EQ(plan_view[0].y_, 20.0);
  EXPECT_DOUBLE_EQ(plan_view[0].hdg_, 0.5);
  EXPECT_DOUBLE_EQ(plan_view[0].length_, 10.0);
  EXPECT_TRUE(std::holds_alternative<strada::ast::Line>(plan_view[0].shape_));

  // 2. Spiral
  EXPECT_DOUBLE_EQ(plan_view[1].s_, 10.0);
  EXPECT_TRUE(std::holds_alternative<strada::ast::Spiral>(plan_view[1].shape_));
  const auto& spiral = std::get<strada::ast::Spiral>(plan_view[1].shape_);
  EXPECT_DOUBLE_EQ(spiral.curv_start_, 0.0);
  EXPECT_DOUBLE_EQ(spiral.curv_end_, 0.1);

  // 3. Arc
  EXPECT_DOUBLE_EQ(plan_view[2].s_, 25.0);
  EXPECT_TRUE(std::holds_alternative<strada::ast::Arc>(plan_view[2].shape_));
  const auto& arc = std::get<strada::ast::Arc>(plan_view[2].shape_);
  EXPECT_DOUBLE_EQ(arc.curvature_, 0.05);

  // 4. Poly3
  EXPECT_DOUBLE_EQ(plan_view[3].s_, 45.0);
  EXPECT_TRUE(std::holds_alternative<strada::ast::Poly3>(plan_view[3].shape_));
  const auto& poly3 = std::get<strada::ast::Poly3>(plan_view[3].shape_);
  EXPECT_DOUBLE_EQ(poly3.a_, 1.0);
  EXPECT_DOUBLE_EQ(poly3.b_, 2.0);
  EXPECT_DOUBLE_EQ(poly3.c_, 3.0);
  EXPECT_DOUBLE_EQ(poly3.d_, 4.0);

  // 5. ParamPoly3
  EXPECT_DOUBLE_EQ(plan_view[4].s_, 70.0);
  EXPECT_TRUE(std::holds_alternative<strada::ast::ParamPoly3>(plan_view[4].shape_));
  const auto& param_poly3 = std::get<strada::ast::ParamPoly3>(plan_view[4].shape_);
  EXPECT_DOUBLE_EQ(param_poly3.a_u_, 1.1);
  EXPECT_DOUBLE_EQ(param_poly3.b_u_, 1.2);
  EXPECT_DOUBLE_EQ(param_poly3.c_u_, 1.3);
  EXPECT_DOUBLE_EQ(param_poly3.d_u_, 1.4);
  EXPECT_DOUBLE_EQ(param_poly3.a_v_, 2.1);
  EXPECT_DOUBLE_EQ(param_poly3.b_v_, 2.2);
  EXPECT_DOUBLE_EQ(param_poly3.c_v_, 2.3);
  EXPECT_DOUBLE_EQ(param_poly3.d_v_, 2.4);
  EXPECT_EQ(param_poly3.p_range_, strada::ast::PRange::ArcLength);
}

TEST(ParserTest, ParseLanesAndProfiles) {
  // Arrange
  std::string_view xml = R"(<?xml version="1.0" encoding="utf-8"?>
<OpenDRIVE>
    <header revMajor="1" revMinor="9" name="Test Lanes Map" version="1.0" date="2026-06-14T09:00:00" north="100.0" south="-100.0" east="200.0" west="-200.0" vendor="Strada Vendor">
        <geoReference><![CDATA[+proj=utm +zone=32 +datum=WGS84]]></geoReference>
    </header>
    <road id="1" length="100.0" junction="-1">
        <elevationProfile>
            <elevation s="0.0" a="1.0" b="2.0" c="3.0" d="4.0"/>
            <elevation s="50.0" a="5.0" b="6.0" c="7.0" d="8.0"/>
        </elevationProfile>
        <lateralProfile>
            <superelevation s="0.0" a="0.1" b="0.2" c="0.3" d="0.4"/>
            <shape s="10.0" t="-2.0" a="1.1" b="1.2" c="1.3" d="1.4"/>
        </lateralProfile>
        <lanes>
            <laneOffset s="0.0" a="0.5" b="0.6" c="0.7" d="0.8"/>
            <laneSection s="0.0">
                <left>
                    <lane id="1" type="driving" level="true">
                        <link>
                            <predecessor id="2"/>
                            <successor id="3"/>
                        </link>
                        <width sOffset="0.0" a="3.0" b="0.1" c="0.0" d="0.0"/>
                        <height sOffset="0.0" inner="0.0" outer="0.1"/>
                    </lane>
                </left>
                <center>
                    <lane id="0" type="border" level="false"/>
                </center>
                <right>
                    <lane id="-1" type="driving" level="false">
                        <link>
                            <predecessor id="-2"/>
                        </link>
                        <width sOffset="1.0" a="3.2" b="0.2" c="0.0" d="0.0"/>
                    </lane>
                </right>
            </laneSection>
        </lanes>
    </road>
</OpenDRIVE>
)";

  // Act
  auto opendrive = strada::parser::ParseString(xml);

  // Assert
  ASSERT_EQ(opendrive.roads_.size(), 1);
  const auto& road = opendrive.roads_[0];

  // Elevation Profile
  const auto& elevations = road.elevation_profile_.elevations_;
  ASSERT_EQ(elevations.size(), 2);
  EXPECT_DOUBLE_EQ(elevations[0].s_, 0.0);
  EXPECT_DOUBLE_EQ(elevations[0].a_, 1.0);
  EXPECT_DOUBLE_EQ(elevations[1].s_, 50.0);
  EXPECT_DOUBLE_EQ(elevations[1].d_, 8.0);

  // Lateral Profile
  const auto& superelevations = road.lateral_profile_.superelevations_;
  ASSERT_EQ(superelevations.size(), 1);
  EXPECT_DOUBLE_EQ(superelevations[0].s_, 0.0);
  EXPECT_DOUBLE_EQ(superelevations[0].a_, 0.1);

  const auto& shapes = road.lateral_profile_.shapes_;
  ASSERT_EQ(shapes.size(), 1);
  EXPECT_DOUBLE_EQ(shapes[0].s_, 10.0);
  EXPECT_DOUBLE_EQ(shapes[0].t_, -2.0);
  EXPECT_DOUBLE_EQ(shapes[0].a_, 1.1);

  // Lanes
  const auto& lanes = road.lanes_;
  ASSERT_EQ(lanes.offsets_.size(), 1);
  EXPECT_DOUBLE_EQ(lanes.offsets_[0].s_, 0.0);
  EXPECT_DOUBLE_EQ(lanes.offsets_[0].a_, 0.5);

  ASSERT_EQ(lanes.sections_.size(), 1);
  const auto& section = lanes.sections_[0];
  EXPECT_DOUBLE_EQ(section.s_, 0.0);

  // Left Lane
  ASSERT_EQ(section.left_.size(), 1);
  EXPECT_EQ(section.left_[0].id_, 1);
  EXPECT_EQ(section.left_[0].type_, "driving");
  EXPECT_TRUE(section.left_[0].level_);
  ASSERT_TRUE(section.left_[0].predecessor_.has_value());
  EXPECT_EQ(section.left_[0].predecessor_.value_or(0), 2);
  ASSERT_TRUE(section.left_[0].successor_.has_value());
  EXPECT_EQ(section.left_[0].successor_.value_or(0), 3);
  ASSERT_EQ(section.left_[0].widths_.size(), 1);
  EXPECT_DOUBLE_EQ(section.left_[0].widths_[0].s_offset_, 0.0);
  EXPECT_DOUBLE_EQ(section.left_[0].widths_[0].a_, 3.0);
  ASSERT_EQ(section.left_[0].heights_.size(), 1);
  EXPECT_DOUBLE_EQ(section.left_[0].heights_[0].s_offset_, 0.0);
  EXPECT_DOUBLE_EQ(section.left_[0].heights_[0].outer_, 0.1);

  // Center Lane
  ASSERT_EQ(section.center_.size(), 1);
  EXPECT_EQ(section.center_[0].id_, 0);
  EXPECT_EQ(section.center_[0].type_, "border");
  EXPECT_FALSE(section.center_[0].level_);

  // Right Lane
  ASSERT_EQ(section.right_.size(), 1);
  EXPECT_EQ(section.right_[0].id_, -1);
  EXPECT_EQ(section.right_[0].type_, "driving");
  EXPECT_FALSE(section.right_[0].level_);
  ASSERT_TRUE(section.right_[0].predecessor_.has_value());
  EXPECT_EQ(section.right_[0].predecessor_.value_or(0), -2);
  EXPECT_FALSE(section.right_[0].successor_.has_value());
  ASSERT_EQ(section.right_[0].widths_.size(), 1);
  EXPECT_DOUBLE_EQ(section.right_[0].widths_[0].s_offset_, 1.0);
  EXPECT_DOUBLE_EQ(section.right_[0].widths_[0].a_, 3.2);
}
