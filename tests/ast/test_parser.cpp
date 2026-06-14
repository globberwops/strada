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
  const auto& header = opendrive.header;
  EXPECT_EQ(header.rev_major, 1);
  EXPECT_EQ(header.rev_minor, 9);
  EXPECT_EQ(header.name, "Test Map");
  EXPECT_EQ(header.version, "1.0");
  EXPECT_EQ(header.date, "2026-06-14T09:00:00");
  EXPECT_DOUBLE_EQ(header.north, 100.0);
  EXPECT_DOUBLE_EQ(header.south, -100.0);
  EXPECT_DOUBLE_EQ(header.east, 200.0);
  EXPECT_DOUBLE_EQ(header.west, -200.0);
  EXPECT_EQ(header.vendor, "Strada Vendor");
  EXPECT_EQ(header.geo_reference, "+proj=utm +zone=32 +datum=WGS84");
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
  const auto& header = opendrive.header;
  EXPECT_EQ(header.rev_major, 1);
  EXPECT_EQ(header.rev_minor, 9);
  EXPECT_EQ(header.name, "Test Map");
  EXPECT_EQ(header.geo_reference, "+proj=utm +zone=32 +datum=WGS84");
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
  const auto& roads = opendrive.roads;
  ASSERT_EQ(roads.size(), 2);

  EXPECT_EQ(roads[0].id, "1");
  EXPECT_DOUBLE_EQ(roads[0].length, 10.0);
  EXPECT_EQ(roads[0].junction, "-1");
  EXPECT_EQ(roads[0].rule, strada::ast::TrafficRule::RHT);
  EXPECT_EQ(roads[0].name, "Road 1");

  EXPECT_EQ(roads[1].id, "2");
  EXPECT_DOUBLE_EQ(roads[1].length, 25.5);
  EXPECT_EQ(roads[1].junction, "42");
  EXPECT_EQ(roads[1].rule, strada::ast::TrafficRule::LHT);
  EXPECT_EQ(roads[1].name, "");
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
  ASSERT_EQ(opendrive.roads.size(), 1);
  const auto& road = opendrive.roads[0];
  const auto& plan_view = road.plan_view;
  ASSERT_EQ(plan_view.size(), 5);

  // 1. Line
  EXPECT_DOUBLE_EQ(plan_view[0].s, 0.0);
  EXPECT_DOUBLE_EQ(plan_view[0].x, 10.0);
  EXPECT_DOUBLE_EQ(plan_view[0].y, 20.0);
  EXPECT_DOUBLE_EQ(plan_view[0].hdg, 0.5);
  EXPECT_DOUBLE_EQ(plan_view[0].length, 10.0);
  EXPECT_TRUE(std::holds_alternative<strada::ast::Line>(plan_view[0].shape));

  // 2. Spiral
  EXPECT_DOUBLE_EQ(plan_view[1].s, 10.0);
  EXPECT_TRUE(std::holds_alternative<strada::ast::Spiral>(plan_view[1].shape));
  const auto& spiral = std::get<strada::ast::Spiral>(plan_view[1].shape);
  EXPECT_DOUBLE_EQ(spiral.curv_start, 0.0);
  EXPECT_DOUBLE_EQ(spiral.curv_end, 0.1);

  // 3. Arc
  EXPECT_DOUBLE_EQ(plan_view[2].s, 25.0);
  EXPECT_TRUE(std::holds_alternative<strada::ast::Arc>(plan_view[2].shape));
  const auto& arc = std::get<strada::ast::Arc>(plan_view[2].shape);
  EXPECT_DOUBLE_EQ(arc.curvature, 0.05);

  // 4. Poly3
  EXPECT_DOUBLE_EQ(plan_view[3].s, 45.0);
  EXPECT_TRUE(std::holds_alternative<strada::ast::Poly3>(plan_view[3].shape));
  const auto& poly3 = std::get<strada::ast::Poly3>(plan_view[3].shape);
  EXPECT_DOUBLE_EQ(poly3.a, 1.0);
  EXPECT_DOUBLE_EQ(poly3.b, 2.0);
  EXPECT_DOUBLE_EQ(poly3.c, 3.0);
  EXPECT_DOUBLE_EQ(poly3.d, 4.0);

  // 5. ParamPoly3
  EXPECT_DOUBLE_EQ(plan_view[4].s, 70.0);
  EXPECT_TRUE(std::holds_alternative<strada::ast::ParamPoly3>(plan_view[4].shape));
  const auto& param_poly3 = std::get<strada::ast::ParamPoly3>(plan_view[4].shape);
  EXPECT_DOUBLE_EQ(param_poly3.a_u, 1.1);
  EXPECT_DOUBLE_EQ(param_poly3.b_u, 1.2);
  EXPECT_DOUBLE_EQ(param_poly3.c_u, 1.3);
  EXPECT_DOUBLE_EQ(param_poly3.d_u, 1.4);
  EXPECT_DOUBLE_EQ(param_poly3.a_v, 2.1);
  EXPECT_DOUBLE_EQ(param_poly3.b_v, 2.2);
  EXPECT_DOUBLE_EQ(param_poly3.c_v, 2.3);
  EXPECT_DOUBLE_EQ(param_poly3.d_v, 2.4);
  EXPECT_EQ(param_poly3.p_range, strada::ast::PRange::ArcLength);
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
  ASSERT_EQ(opendrive.roads.size(), 1);
  const auto& road = opendrive.roads[0];

  // Elevation Profile
  const auto& elevations = road.elevation_profile.elevations;
  ASSERT_EQ(elevations.size(), 2);
  EXPECT_DOUBLE_EQ(elevations[0].s, 0.0);
  EXPECT_DOUBLE_EQ(elevations[0].a, 1.0);
  EXPECT_DOUBLE_EQ(elevations[1].s, 50.0);
  EXPECT_DOUBLE_EQ(elevations[1].d, 8.0);

  // Lateral Profile
  const auto& superelevations = road.lateral_profile.superelevations;
  ASSERT_EQ(superelevations.size(), 1);
  EXPECT_DOUBLE_EQ(superelevations[0].s, 0.0);
  EXPECT_DOUBLE_EQ(superelevations[0].a, 0.1);

  const auto& shapes = road.lateral_profile.shapes;
  ASSERT_EQ(shapes.size(), 1);
  EXPECT_DOUBLE_EQ(shapes[0].s, 10.0);
  EXPECT_DOUBLE_EQ(shapes[0].t, -2.0);
  EXPECT_DOUBLE_EQ(shapes[0].a, 1.1);

  // Lanes
  const auto& lanes = road.lanes;
  ASSERT_EQ(lanes.offsets.size(), 1);
  EXPECT_DOUBLE_EQ(lanes.offsets[0].s, 0.0);
  EXPECT_DOUBLE_EQ(lanes.offsets[0].a, 0.5);

  ASSERT_EQ(lanes.sections.size(), 1);
  const auto& section = lanes.sections[0];
  EXPECT_DOUBLE_EQ(section.s, 0.0);

  // Left Lane
  ASSERT_EQ(section.left.size(), 1);
  EXPECT_EQ(section.left[0].id, 1);
  EXPECT_EQ(section.left[0].type, "driving");
  EXPECT_TRUE(section.left[0].level);
  ASSERT_TRUE(section.left[0].predecessor.has_value());
  EXPECT_EQ(section.left[0].predecessor.value_or(0), 2);
  ASSERT_TRUE(section.left[0].successor.has_value());
  EXPECT_EQ(section.left[0].successor.value_or(0), 3);
  ASSERT_EQ(section.left[0].widths.size(), 1);
  EXPECT_DOUBLE_EQ(section.left[0].widths[0].s_offset, 0.0);
  EXPECT_DOUBLE_EQ(section.left[0].widths[0].a, 3.0);
  ASSERT_EQ(section.left[0].heights.size(), 1);
  EXPECT_DOUBLE_EQ(section.left[0].heights[0].s_offset, 0.0);
  EXPECT_DOUBLE_EQ(section.left[0].heights[0].outer, 0.1);

  // Center Lane
  ASSERT_EQ(section.center.size(), 1);
  EXPECT_EQ(section.center[0].id, 0);
  EXPECT_EQ(section.center[0].type, "border");
  EXPECT_FALSE(section.center[0].level);

  // Right Lane
  ASSERT_EQ(section.right.size(), 1);
  EXPECT_EQ(section.right[0].id, -1);
  EXPECT_EQ(section.right[0].type, "driving");
  EXPECT_FALSE(section.right[0].level);
  ASSERT_TRUE(section.right[0].predecessor.has_value());
  EXPECT_EQ(section.right[0].predecessor.value_or(0), -2);
  EXPECT_FALSE(section.right[0].successor.has_value());
  ASSERT_EQ(section.right[0].widths.size(), 1);
  EXPECT_DOUBLE_EQ(section.right[0].widths[0].s_offset, 1.0);
  EXPECT_DOUBLE_EQ(section.right[0].widths[0].a, 3.2);
}
