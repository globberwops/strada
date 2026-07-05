#include <gtest/gtest.h>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <strada/parser/conversions.hpp>
#include <strada/parser/parser.hpp>
#include <string>

namespace {

auto ReadFileToString(const std::filesystem::path& path) -> std::string {
  std::ifstream file_stream(path, std::ios::in | std::ios::binary);
  if (!file_stream) {
    throw std::runtime_error("Cannot open file: " + path.string());
  }
  std::string contents;
  file_stream.seekg(0, std::ios::end);
  contents.resize(static_cast<std::string::size_type>(file_stream.tellg()));
  file_stream.seekg(0, std::ios::beg);
  file_stream.read(contents.data(), static_cast<std::streamsize>(contents.size()));
  return contents;
}

}  // namespace

TEST(ParserTest, ParseMinimalHeaderFromString) {
  // Arrange
  std::filesystem::path data_dir = STRADA_TEST_DATA_DIR;
  std::string xml = ReadFileToString(data_dir / "minimal_header.xodr");

  // Act
  auto ast_tree = strada::parser::ParseString(xml);

  // Assert
  const auto& header = ast_tree.header;
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
  std::filesystem::path data_dir = STRADA_TEST_DATA_DIR;
  std::filesystem::path file_path = data_dir / "minimal_header.xodr";

  // Act
  auto ast_tree = strada::parser::ParseFile(file_path);

  // Assert
  const auto& header = ast_tree.header;
  EXPECT_EQ(header.rev_major, 1);
  EXPECT_EQ(header.rev_minor, 9);
  EXPECT_EQ(header.name, "Test Map");
  EXPECT_EQ(header.geo_reference, "+proj=utm +zone=32 +datum=WGS84");
}

TEST(ParserTest, ParseRoadsFromString) {
  // Arrange
  std::filesystem::path data_dir = STRADA_TEST_DATA_DIR;
  std::string xml = ReadFileToString(data_dir / "roads.xodr");

  // Act
  auto ast_tree = strada::parser::ParseString(xml);

  // Assert
  const auto& roads = ast_tree.roads;
  ASSERT_EQ(roads.size(), 2);

  EXPECT_EQ(roads[0].id, "1");
  EXPECT_DOUBLE_EQ(roads[0].length, 10.0);
  EXPECT_EQ(roads[0].junction, "-1");
  EXPECT_EQ(roads[0].rule, strada::ast::TrafficRule::kRht);
  ASSERT_TRUE(roads[0].name.has_value());
  EXPECT_EQ(*roads[0].name, "Road 1");

  EXPECT_EQ(roads[1].id, "2");
  EXPECT_DOUBLE_EQ(roads[1].length, 25.5);
  EXPECT_EQ(roads[1].junction, "42");
  EXPECT_EQ(roads[1].rule, strada::ast::TrafficRule::kLht);
  EXPECT_EQ(roads[1].name, std::nullopt);
}

TEST(ParserTest, ParseGeometryFromPlanView) {
  // Arrange
  std::filesystem::path data_dir = STRADA_TEST_DATA_DIR;
  std::string xml = ReadFileToString(data_dir / "geometry.xodr");

  // Act
  auto ast_tree = strada::parser::ParseString(xml);

  // Assert
  ASSERT_EQ(ast_tree.roads.size(), 1);
  const auto& road = ast_tree.roads[0];
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
  EXPECT_EQ(param_poly3.p_range, strada::ast::PRange::kArcLength);
}

TEST(ParserTest, ParseLanesAndProfiles) {
  // Arrange
  std::filesystem::path data_dir = STRADA_TEST_DATA_DIR;
  std::string xml = ReadFileToString(data_dir / "lanes_and_profiles.xodr");

  // Act
  auto ast_tree = strada::parser::ParseString(xml);

  // Assert
  ASSERT_EQ(ast_tree.roads.size(), 1);
  const auto& road = ast_tree.roads[0];

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
  EXPECT_EQ(section.left[0].type, strada::ast::LaneType::kDriving);
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
  EXPECT_EQ(section.center[0].type, strada::ast::LaneType::kBorder);
  EXPECT_FALSE(section.center[0].level);

  // Right Lane
  ASSERT_EQ(section.right.size(), 1);
  EXPECT_EQ(section.right[0].id, -1);
  EXPECT_EQ(section.right[0].type, strada::ast::LaneType::kDriving);
  EXPECT_FALSE(section.right[0].level);
  ASSERT_TRUE(section.right[0].predecessor.has_value());
  EXPECT_EQ(section.right[0].predecessor.value_or(0), -2);
  EXPECT_FALSE(section.right[0].successor.has_value());
  ASSERT_EQ(section.right[0].widths.size(), 1);
  EXPECT_DOUBLE_EQ(section.right[0].widths[0].s_offset, 1.0);
  EXPECT_DOUBLE_EQ(section.right[0].widths[0].a, 3.2);
}

TEST(ParserTest, ParseJunctions) {
  // Arrange
  std::filesystem::path data_dir = STRADA_TEST_DATA_DIR;
  std::string xml = ReadFileToString(data_dir / "junctions.xodr");

  // Act
  auto ast_tree = strada::parser::ParseString(xml);

  // Assert
  ASSERT_EQ(ast_tree.junctions.size(), 1);
  const auto& junction = ast_tree.junctions[0];
  EXPECT_EQ(junction.id, "1");
  ASSERT_TRUE(junction.name.has_value());
  EXPECT_EQ(*junction.name, "Main Junction");
  EXPECT_EQ(junction.type, strada::ast::JunctionType::kCommon);

  ASSERT_EQ(junction.connections.size(), 2);

  const auto& conn0 = junction.connections[0];
  EXPECT_EQ(conn0.id, "0");
  EXPECT_EQ(conn0.incoming_road, "10");
  EXPECT_EQ(conn0.connecting_road, "20");
  EXPECT_EQ(conn0.contact_point, strada::ast::ContactPoint::kStart);
  ASSERT_EQ(conn0.lane_links.size(), 2);
  EXPECT_EQ(conn0.lane_links[0].from, -1);
  EXPECT_EQ(conn0.lane_links[0].to, -1);
  EXPECT_EQ(conn0.lane_links[1].from, -2);
  EXPECT_EQ(conn0.lane_links[1].to, -2);

  const auto& conn1 = junction.connections[1];
  EXPECT_EQ(conn1.id, "1");
  EXPECT_EQ(conn1.incoming_road, "30");
  EXPECT_EQ(conn1.connecting_road, "40");
  EXPECT_EQ(conn1.contact_point, strada::ast::ContactPoint::kEnd);
  ASSERT_EQ(conn1.lane_links.size(), 1);
  EXPECT_EQ(conn1.lane_links[0].from, 1);
  EXPECT_EQ(conn1.lane_links[0].to, 1);
}

TEST(ParserTest, ParseOtherJunctionTypes) {
  // Arrange
  const std::string kXml = R"(<?xml version="1.0" encoding="utf-8"?>
<OpenDRIVE>
    <header revMajor="1" revMinor="9" name="Junction Types Map" version="1.0" date="2026-06-21T12:00:00" north="100.0" south="-100.0" east="200.0" west="-200.0" vendor="Strada Vendor"/>
    <junction id="1" name="Crossing Junction" type="crossing"/>
    <junction id="2" name="Direct Junction" type="direct"/>
    <junction id="3" name="Virtual Junction" type="virtual"/>
</OpenDRIVE>)";

  // Act
  auto ast_tree = strada::parser::ParseString(kXml);

  // Assert
  ASSERT_EQ(ast_tree.junctions.size(), 3);

  EXPECT_EQ(ast_tree.junctions[0].id, "1");
  ASSERT_TRUE(ast_tree.junctions[0].name.has_value());
  EXPECT_EQ(*ast_tree.junctions[0].name, "Crossing Junction");
  EXPECT_EQ(ast_tree.junctions[0].type, strada::ast::JunctionType::kCrossing);

  EXPECT_EQ(ast_tree.junctions[1].id, "2");
  ASSERT_TRUE(ast_tree.junctions[1].name.has_value());
  EXPECT_EQ(*ast_tree.junctions[1].name, "Direct Junction");
  EXPECT_EQ(ast_tree.junctions[1].type, strada::ast::JunctionType::kDirect);

  EXPECT_EQ(ast_tree.junctions[2].id, "3");
  ASSERT_TRUE(ast_tree.junctions[2].name.has_value());
  EXPECT_EQ(*ast_tree.junctions[2].name, "Virtual Junction");
  EXPECT_EQ(ast_tree.junctions[2].type, strada::ast::JunctionType::kVirtual);
}

TEST(ParserTest, ParseJunctionMissingTypeAndName) {
  // Arrange
  const std::string kXml = R"(<?xml version="1.0" encoding="utf-8"?>
<OpenDRIVE>
    <header revMajor="1" revMinor="9" name="Junction Types Map" version="1.0" date="2026-06-21T12:00:00" north="100.0" south="-100.0" east="200.0" west="-200.0" vendor="Strada Vendor"/>
    <junction id="1"/>
</OpenDRIVE>)";

  // Act
  auto ast_tree = strada::parser::ParseString(kXml);

  // Assert
  ASSERT_EQ(ast_tree.junctions.size(), 1);
  const auto& junction = ast_tree.junctions[0];
  EXPECT_EQ(junction.id, "1");
  EXPECT_EQ(junction.name, std::nullopt);
  EXPECT_EQ(junction.type, strada::ast::JunctionType::kCommon);
}

TEST(ParserTest, ParseJunctionInvalidTypeThrows) {
  // Arrange
  const std::string kXml = R"(<?xml version="1.0" encoding="utf-8"?>
<OpenDRIVE>
    <header revMajor="1" revMinor="9" name="Junction Types Map" version="1.0" date="2026-06-21T12:00:00" north="100.0" south="-100.0" east="200.0" west="-200.0" vendor="Strada Vendor"/>
    <junction id="1" name="Invalid Junction" type="invalid_type"/>
</OpenDRIVE>)";

  // Act & Assert
  EXPECT_THROW(strada::parser::ParseString(kXml), strada::parser::InvalidAttributeError);
}

TEST(ParserTest, ParseJunctionBoundary) {
  // Arrange
  std::filesystem::path data_dir = STRADA_TEST_DATA_DIR;
  std::string xml = ReadFileToString(data_dir / "junction_boundary.xodr");

  // Act
  auto ast_tree = strada::parser::ParseString(xml);

  // Assert
  ASSERT_EQ(ast_tree.junctions.size(), 1);
  const auto& junction = ast_tree.junctions[0];
  EXPECT_EQ(junction.id, "1");

  ASSERT_TRUE(junction.boundary.has_value());
  const auto& boundary = *junction.boundary;
  ASSERT_EQ(boundary.segments.size(), 2);

  // segment type="lane" roadId="8" boundaryLane="-2" sStart="begin" sEnd="end"
  const auto& seg0 = boundary.segments[0];
  EXPECT_EQ(seg0.type, strada::ast::JunctionSegmentType::kLane);
  EXPECT_EQ(seg0.road_id, "8");
  ASSERT_TRUE(seg0.boundary_lane.has_value());
  EXPECT_EQ(*seg0.boundary_lane, -2);
  EXPECT_DOUBLE_EQ(seg0.s_start, 0.0);
  EXPECT_TRUE(std::isinf(seg0.s_end));

  // segment type="lane" roadId="32" boundaryLane="-2" sStart="begin" sEnd="40.0"
  const auto& seg1 = boundary.segments[1];
  EXPECT_EQ(seg1.type, strada::ast::JunctionSegmentType::kLane);
  EXPECT_EQ(seg1.road_id, "32");
  ASSERT_TRUE(seg1.boundary_lane.has_value());
  EXPECT_EQ(*seg1.boundary_lane, -2);
  EXPECT_DOUBLE_EQ(seg1.s_start, 0.0);
  EXPECT_DOUBLE_EQ(seg1.s_end, 40.0);
}

TEST(ParserTest, ParseJunctionJointBoundary) {
  // Arrange
  const std::string kXml = R"(<?xml version="1.0" encoding="utf-8"?>
<OpenDRIVE>
    <header revMajor="1" revMinor="9" name="Joint Boundary Test Map" version="1.0" date="2026-06-21T12:00:00" north="100.0" south="-100.0" east="200.0" west="-200.0" vendor="Strada Vendor"/>
    <road id="1" length="10.0" junction="-1">
        <planView><geometry s="0.0" x="0.0" y="0.0" hdg="0.0" length="10.0"><line /></geometry></planView>
        <lanes><laneSection s="0.0"><center><lane id="0" type="border" level="false"/></center></laneSection></lanes>
    </road>
    <junction id="1" name="Main Junction" type="default">
        <boundary>
            <segment type="joint" roadId="1" contactPoint="end" jointLaneStart="-1" jointLaneEnd="-2" transitionLength="1.5"/>
            <segment type="joint" roadId="1" contactPoint="start"/>
        </boundary>
    </junction>
</OpenDRIVE>)";

  // Act
  auto ast_tree = strada::parser::ParseString(kXml);

  // Assert
  ASSERT_EQ(ast_tree.junctions.size(), 1);
  const auto& junction = ast_tree.junctions[0];
  ASSERT_TRUE(junction.boundary.has_value());
  const auto& boundary = *junction.boundary;
  ASSERT_EQ(boundary.segments.size(), 2);

  const auto& seg0 = boundary.segments[0];
  EXPECT_EQ(seg0.type, strada::ast::JunctionSegmentType::kJoint);
  EXPECT_EQ(seg0.road_id, "1");
  EXPECT_EQ(seg0.contact_point, strada::ast::ContactPoint::kEnd);
  ASSERT_TRUE(seg0.joint_lane_start.has_value());
  EXPECT_EQ(*seg0.joint_lane_start, -1);
  ASSERT_TRUE(seg0.joint_lane_end.has_value());
  EXPECT_EQ(*seg0.joint_lane_end, -2);
  EXPECT_DOUBLE_EQ(seg0.transition_length, 1.5);

  const auto& seg1 = boundary.segments[1];
  EXPECT_EQ(seg1.type, strada::ast::JunctionSegmentType::kJoint);
  EXPECT_EQ(seg1.road_id, "1");
  EXPECT_EQ(seg1.contact_point, strada::ast::ContactPoint::kStart);
  EXPECT_FALSE(seg1.joint_lane_start.has_value());
  EXPECT_FALSE(seg1.joint_lane_end.has_value());
  EXPECT_DOUBLE_EQ(seg1.transition_length, 0.0);
}

TEST(ParserTest, ParseExtensions) {
  // Arrange
  std::filesystem::path data_dir = STRADA_TEST_DATA_DIR;
  std::string xml = ReadFileToString(data_dir / "extensions.xodr");

  // Act
  auto ast_tree = strada::parser::ParseString(xml);

  // Assert – header captures unknown attributes and userData
  const auto& header_ext = ast_tree.header.extensions;
  ASSERT_EQ(header_ext.attributes.size(), 1);
  EXPECT_EQ(header_ext.attributes.at("customKey"), "customValue");
  ASSERT_EQ(header_ext.user_data.size(), 1);
  EXPECT_NE(header_ext.user_data[0].find("someVendorData"), std::string::npos);

  // Assert – road captures unknown attributes and userData
  ASSERT_EQ(ast_tree.roads.size(), 1);
  const auto& road_ext = ast_tree.roads[0].extensions;
  ASSERT_EQ(road_ext.attributes.size(), 1);
  EXPECT_EQ(road_ext.attributes.at("customAttr"), "roadExtra");
  ASSERT_EQ(road_ext.user_data.size(), 1);
  EXPECT_NE(road_ext.user_data[0].find("roadVendorTag"), std::string::npos);
}

TEST(ParserTest, ThrowsXmlParseErrorOnMalformedXml) {
  // Arrange
  const std::string kMalformedXml = "<not valid xml <<< >";

  // Act & Assert
  EXPECT_THROW(strada::parser::ParseString(kMalformedXml), strada::parser::XmlParseError);
}

TEST(ParserTest, ThrowsMissingElementErrorOnMissingRoot) {
  // Arrange
  const std::string kXml = "<NotOpenDRIVE />";

  // Act & Assert
  EXPECT_THROW(strada::parser::ParseString(kXml), strada::parser::MissingElementError);
}

TEST(ParserTest, ThrowsMissingElementErrorOnMissingHeader) {
  // Arrange
  const std::string kXml = R"(<?xml version="1.0"?><OpenDRIVE></OpenDRIVE>)";

  // Act & Assert
  EXPECT_THROW(strada::parser::ParseString(kXml), strada::parser::MissingElementError);
}

TEST(ParserTest, ThrowsMissingElementErrorOnMissingRoadId) {
  // Arrange
  std::filesystem::path data_dir = STRADA_TEST_DATA_DIR;
  std::string xml = ReadFileToString(data_dir / "error_missing_road_id.xodr");

  // Act & Assert
  EXPECT_THROW(strada::parser::ParseString(xml), strada::parser::MissingElementError);
}

TEST(ParserTest, ThrowsMissingElementErrorOnMissingPlanView) {
  // Arrange
  std::filesystem::path data_dir = STRADA_TEST_DATA_DIR;
  std::string xml = ReadFileToString(data_dir / "error_missing_plan_view.xodr");

  // Act & Assert
  EXPECT_THROW(strada::parser::ParseString(xml), strada::parser::MissingElementError);
}

TEST(ParserTest, ErrorMessageContainsRoadIdOnMissingPlanView) {
  // Arrange
  std::filesystem::path data_dir = STRADA_TEST_DATA_DIR;
  std::string xml = ReadFileToString(data_dir / "error_missing_plan_view.xodr");

  // Act & Assert
  try {
    strada::parser::ParseString(xml);
    FAIL() << "Expected MissingElementError to be thrown";
  } catch (const strada::parser::MissingElementError& err) {
    EXPECT_NE(std::string(err.what()).find('1'), std::string::npos);
  }
}

TEST(ParserTest, ParseCrossSectionSurface) {
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
          <lane id="0" type="none"/>
        </center>
      </laneSection>
    </lanes>
  </road>
</OpenDRIVE>)";

  // Act
  auto ast_tree = strada::parser::ParseString(kXml);

  // Assert
  ASSERT_EQ(ast_tree.roads.size(), 1);
  const auto& road = ast_tree.roads[0];
  ASSERT_TRUE(road.lateral_profile.cross_section_surface.has_value());
  const auto& css = *road.lateral_profile.cross_section_surface;

  // Verify tOffset
  ASSERT_EQ(css.t_offset.size(), 1);
  EXPECT_DOUBLE_EQ(css.t_offset[0].s, 0.0);
  EXPECT_DOUBLE_EQ(css.t_offset[0].a, -0.375);
  EXPECT_DOUBLE_EQ(css.t_offset[0].b, 0.0);

  // Verify strips
  ASSERT_EQ(css.strips.size(), 1);
  const auto& strip = css.strips[0];
  EXPECT_EQ(strip.id, 1);
  EXPECT_EQ(strip.mode, strada::ast::StripMode::kIndependent);

  // Verify constant coefficients
  ASSERT_EQ(strip.constant.size(), 1);
  EXPECT_DOUBLE_EQ(strip.constant[0].s, 0.0);
  EXPECT_DOUBLE_EQ(strip.constant[0].a, 0.45);
  EXPECT_DOUBLE_EQ(strip.constant[0].b, 0.0);
}

TEST(ParserTest, ParseCrossSectionSurfaceWithRelativeMode) {
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
        <surfaceStrips>
          <strip id="1" mode="relative"/>
        </surfaceStrips>
      </crossSectionSurface>
    </lateralProfile>
    <lanes>
      <laneSection s="0.0">
        <center>
          <lane id="0" type="none"/>
        </center>
      </laneSection>
    </lanes>
  </road>
</OpenDRIVE>)";

  // Act
  auto ast_tree = strada::parser::ParseString(kXml);

  // Assert
  ASSERT_EQ(ast_tree.roads.size(), 1);
  const auto& road = ast_tree.roads[0];
  ASSERT_TRUE(road.lateral_profile.cross_section_surface.has_value());
  const auto& css = *road.lateral_profile.cross_section_surface;
  ASSERT_EQ(css.strips.size(), 1);
  EXPECT_EQ(css.strips[0].mode, strada::ast::StripMode::kRelative);
}

TEST(ParserTest, ParseCrossSectionSurfaceWithMissingMode) {
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
        <surfaceStrips>
          <strip id="1"/>
        </surfaceStrips>
      </crossSectionSurface>
    </lateralProfile>
    <lanes>
      <laneSection s="0.0">
        <center>
          <lane id="0" type="none"/>
        </center>
      </laneSection>
    </lanes>
  </road>
</OpenDRIVE>)";

  // Act
  auto ast_tree = strada::parser::ParseString(kXml);

  // Assert
  ASSERT_EQ(ast_tree.roads.size(), 1);
  const auto& road = ast_tree.roads[0];
  ASSERT_TRUE(road.lateral_profile.cross_section_surface.has_value());
  const auto& css = *road.lateral_profile.cross_section_surface;
  ASSERT_EQ(css.strips.size(), 1);
  EXPECT_EQ(css.strips[0].mode, strada::ast::StripMode::kIndependent);
}

TEST(ParserTest, ParseCrossSectionSurfaceThrowsOnInvalidMode) {
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
        <surfaceStrips>
          <strip id="1" mode="invalid_mode"/>
        </surfaceStrips>
      </crossSectionSurface>
    </lateralProfile>
    <lanes>
      <laneSection s="0.0">
        <center>
          <lane id="0" type="none"/>
        </center>
      </laneSection>
    </lanes>
  </road>
</OpenDRIVE>)";

  // Act & Assert
  EXPECT_THROW(strada::parser::ParseString(kXml), strada::parser::InvalidAttributeError);
}

class BridgesAndTunnelsParserTest : public ::testing::Test {};

TEST_F(BridgesAndTunnelsParserTest, ParseBridgesAndTunnels) {
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
    <lateralProfile/>
    <lanes>
      <laneSection s="0.0">
        <center>
          <lane id="0" type="none"/>
        </center>
      </laneSection>
    </lanes>
    <bridge id="b1" s="10.0" length="30.0" name="test_bridge" type="concrete"/>
    <tunnel id="t1" s="50.0" length="40.0" name="test_tunnel" type="standard" lighting="0.8" daylight="0.2"/>
  </road>
</OpenDRIVE>)";

  // Act
  auto ast_tree = strada::parser::ParseString(kXml);

  // Assert
  ASSERT_EQ(ast_tree.roads.size(), 1);
  const auto& road = ast_tree.roads[0];

  // Verify bridges
  ASSERT_EQ(road.bridges.size(), 1);
  const auto& bridge = road.bridges[0];
  EXPECT_EQ(bridge.id, "b1");
  EXPECT_DOUBLE_EQ(bridge.s, 10.0);
  EXPECT_DOUBLE_EQ(bridge.length, 30.0);
  ASSERT_TRUE(bridge.name.has_value());
  EXPECT_EQ(*bridge.name, "test_bridge");
  EXPECT_EQ(bridge.type, strada::ast::BridgeType::kConcrete);

  // Verify tunnels
  ASSERT_EQ(road.tunnels.size(), 1);
  const auto& tunnel = road.tunnels[0];
  EXPECT_EQ(tunnel.id, "t1");
  EXPECT_DOUBLE_EQ(tunnel.s, 50.0);
  EXPECT_DOUBLE_EQ(tunnel.length, 40.0);
  ASSERT_TRUE(tunnel.name.has_value());
  EXPECT_EQ(*tunnel.name, "test_tunnel");
  EXPECT_EQ(tunnel.type, strada::ast::TunnelType::kStandard);
  ASSERT_TRUE(tunnel.lighting.has_value());
  EXPECT_DOUBLE_EQ(*tunnel.lighting, 0.8);
  ASSERT_TRUE(tunnel.daylight.has_value());
  EXPECT_DOUBLE_EQ(*tunnel.daylight, 0.2);
}

TEST_F(BridgesAndTunnelsParserTest, ParseBridgesAndTunnelsWithValiditiesAndExtensions) {
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
    <lateralProfile/>
    <lanes>
      <laneSection s="0.0">
        <center>
          <lane id="0" type="none"/>
        </center>
      </laneSection>
    </lanes>
    <bridge id="b1" s="10.0" length="30.0" name="test_bridge" type="concrete" customBridgeAttr="bridgeExtra">
      <validity fromLane="-2" toLane="-1" layer="temporary"/>
      <validity fromLane="1" toLane="2"/>
      <userData>
        <bridgeVendorTag>val</bridgeVendorTag>
      </userData>
    </bridge>
    <tunnel id="t1" s="50.0" length="40.0" name="test_tunnel" type="standard" lighting="0.8" daylight="0.2" customTunnelAttr="tunnelExtra">
      <validity fromLane="-3" toLane="-1"/>
      <userData>
        <tunnelVendorTag>val2</tunnelVendorTag>
      </userData>
    </tunnel>
  </road>
</OpenDRIVE>)";

  // Act
  auto ast_tree = strada::parser::ParseString(kXml);

  // Assert
  ASSERT_EQ(ast_tree.roads.size(), 1);
  const auto& road = ast_tree.roads[0];

  // Verify bridge validities and extensions
  ASSERT_EQ(road.bridges.size(), 1);
  const auto& bridge = road.bridges[0];
  ASSERT_EQ(bridge.validities.size(), 2);
  EXPECT_EQ(bridge.validities[0].from_lane, -2);
  EXPECT_EQ(bridge.validities[0].to_lane, -1);
  EXPECT_EQ(bridge.validities[0].layer, strada::ast::LayerType::kTemporary);
  EXPECT_EQ(bridge.validities[1].from_lane, 1);
  EXPECT_EQ(bridge.validities[1].to_lane, 2);
  EXPECT_EQ(bridge.validities[1].layer, strada::ast::LayerType::kPermanent);

  EXPECT_EQ(bridge.extensions.attributes.at("customBridgeAttr"), "bridgeExtra");
  ASSERT_EQ(bridge.extensions.user_data.size(), 1);
  EXPECT_NE(bridge.extensions.user_data[0].find("bridgeVendorTag"), std::string::npos);

  // Verify tunnel validities and extensions
  ASSERT_EQ(road.tunnels.size(), 1);
  const auto& tunnel = road.tunnels[0];
  ASSERT_EQ(tunnel.validities.size(), 1);
  EXPECT_EQ(tunnel.validities[0].from_lane, -3);
  EXPECT_EQ(tunnel.validities[0].to_lane, -1);
  EXPECT_EQ(tunnel.validities[0].layer, strada::ast::LayerType::kPermanent);

  EXPECT_EQ(tunnel.extensions.attributes.at("customTunnelAttr"), "tunnelExtra");
  ASSERT_EQ(tunnel.extensions.user_data.size(), 1);
  EXPECT_NE(tunnel.extensions.user_data[0].find("tunnelVendorTag"), std::string::npos);
}

TEST_F(BridgesAndTunnelsParserTest, ThrowsMissingElementErrorOnMissingBridgeId) {
  // Arrange
  const std::string kXml = R"(<?xml version="1.0" standalone="yes"?>
<OpenDRIVE>
  <header revMajor="1" revMinor="9" name="Test Map" version="1.0" date="2026-06-14T09:00:00" north="100.0" south="-100.0" east="200.0" west="-200.0"/>
  <road name="Road 1" length="100.0" id="1" junction="-1" rule="RHT">
    <planView><geometry s="0.0" x="0.0" y="0.0" hdg="0.0" length="100.0"><line/></geometry></planView>
    <lanes><laneSection s="0.0"><center><lane id="0" type="none"/></center></laneSection></lanes>
    <bridge s="10.0" length="30.0"/>
  </road>
</OpenDRIVE>)";

  // Act & Assert
  EXPECT_THROW(strada::parser::ParseString(kXml), strada::parser::MissingElementError);
}

TEST_F(BridgesAndTunnelsParserTest, ThrowsMissingElementErrorOnMissingBridgeS) {
  // Arrange
  const std::string kXml = R"(<?xml version="1.0" standalone="yes"?>
<OpenDRIVE>
  <header revMajor="1" revMinor="9" name="Test Map" version="1.0" date="2026-06-14T09:00:00" north="100.0" south="-100.0" east="200.0" west="-200.0"/>
  <road name="Road 1" length="100.0" id="1" junction="-1" rule="RHT">
    <planView><geometry s="0.0" x="0.0" y="0.0" hdg="0.0" length="100.0"><line/></geometry></planView>
    <lanes><laneSection s="0.0"><center><lane id="0" type="none"/></center></laneSection></lanes>
    <bridge id="b1" length="30.0"/>
  </road>
</OpenDRIVE>)";

  // Act & Assert
  EXPECT_THROW(strada::parser::ParseString(kXml), strada::parser::MissingElementError);
}

TEST_F(BridgesAndTunnelsParserTest, ThrowsMissingElementErrorOnMissingBridgeLength) {
  // Arrange
  const std::string kXml = R"(<?xml version="1.0" standalone="yes"?>
<OpenDRIVE>
  <header revMajor="1" revMinor="9" name="Test Map" version="1.0" date="2026-06-14T09:00:00" north="100.0" south="-100.0" east="200.0" west="-200.0"/>
  <road name="Road 1" length="100.0" id="1" junction="-1" rule="RHT">
    <planView><geometry s="0.0" x="0.0" y="0.0" hdg="0.0" length="100.0"><line/></geometry></planView>
    <lanes><laneSection s="0.0"><center><lane id="0" type="none"/></center></laneSection></lanes>
    <bridge id="b1" s="10.0"/>
  </road>
</OpenDRIVE>)";

  // Act & Assert
  EXPECT_THROW(strada::parser::ParseString(kXml), strada::parser::MissingElementError);
}

TEST_F(BridgesAndTunnelsParserTest, ThrowsInvalidAttributeErrorOnNegativeBridgeS) {
  // Arrange
  const std::string kXml = R"(<?xml version="1.0" standalone="yes"?>
<OpenDRIVE>
  <header revMajor="1" revMinor="9" name="Test Map" version="1.0" date="2026-06-14T09:00:00" north="100.0" south="-100.0" east="200.0" west="-200.0"/>
  <road name="Road 1" length="100.0" id="1" junction="-1" rule="RHT">
    <planView><geometry s="0.0" x="0.0" y="0.0" hdg="0.0" length="100.0"><line/></geometry></planView>
    <lanes><laneSection s="0.0"><center><lane id="0" type="none"/></center></laneSection></lanes>
    <bridge id="b1" s="-10.0" length="30.0" type="concrete"/>
  </road>
</OpenDRIVE>)";

  // Act & Assert
  EXPECT_THROW(strada::parser::ParseString(kXml), strada::parser::InvalidAttributeError);
}

TEST_F(BridgesAndTunnelsParserTest, ThrowsInvalidAttributeErrorOnNegativeBridgeLength) {
  // Arrange
  const std::string kXml = R"(<?xml version="1.0" standalone="yes"?>
<OpenDRIVE>
  <header revMajor="1" revMinor="9" name="Test Map" version="1.0" date="2026-06-14T09:00:00" north="100.0" south="-100.0" east="200.0" west="-200.0"/>
  <road name="Road 1" length="100.0" id="1" junction="-1" rule="RHT">
    <planView><geometry s="0.0" x="0.0" y="0.0" hdg="0.0" length="100.0"><line/></geometry></planView>
    <lanes><laneSection s="0.0"><center><lane id="0" type="none"/></center></laneSection></lanes>
    <bridge id="b1" s="10.0" length="-30.0" type="concrete"/>
  </road>
</OpenDRIVE>)";

  // Act & Assert
  EXPECT_THROW(strada::parser::ParseString(kXml), strada::parser::InvalidAttributeError);
}

TEST_F(BridgesAndTunnelsParserTest, ThrowsMissingElementErrorOnMissingTunnelId) {
  // Arrange
  const std::string kXml = R"(<?xml version="1.0" standalone="yes"?>
<OpenDRIVE>
  <header revMajor="1" revMinor="9" name="Test Map" version="1.0" date="2026-06-14T09:00:00" north="100.0" south="-100.0" east="200.0" west="-200.0"/>
  <road name="Road 1" length="100.0" id="1" junction="-1" rule="RHT">
    <planView><geometry s="0.0" x="0.0" y="0.0" hdg="0.0" length="100.0"><line/></geometry></planView>
    <lanes><laneSection s="0.0"><center><lane id="0" type="none"/></center></laneSection></lanes>
    <tunnel s="10.0" length="30.0"/>
  </road>
</OpenDRIVE>)";

  // Act & Assert
  EXPECT_THROW(strada::parser::ParseString(kXml), strada::parser::MissingElementError);
}

TEST_F(BridgesAndTunnelsParserTest, ThrowsMissingElementErrorOnMissingTunnelS) {
  // Arrange
  const std::string kXml = R"(<?xml version="1.0" standalone="yes"?>
<OpenDRIVE>
  <header revMajor="1" revMinor="9" name="Test Map" version="1.0" date="2026-06-14T09:00:00" north="100.0" south="-100.0" east="200.0" west="-200.0"/>
  <road name="Road 1" length="100.0" id="1" junction="-1" rule="RHT">
    <planView><geometry s="0.0" x="0.0" y="0.0" hdg="0.0" length="100.0"><line/></geometry></planView>
    <lanes><laneSection s="0.0"><center><lane id="0" type="none"/></center></laneSection></lanes>
    <tunnel id="t1" length="30.0"/>
  </road>
</OpenDRIVE>)";

  // Act & Assert
  EXPECT_THROW(strada::parser::ParseString(kXml), strada::parser::MissingElementError);
}

TEST_F(BridgesAndTunnelsParserTest, ThrowsMissingElementErrorOnMissingTunnelLength) {
  // Arrange
  const std::string kXml = R"(<?xml version="1.0" standalone="yes"?>
<OpenDRIVE>
  <header revMajor="1" revMinor="9" name="Test Map" version="1.0" date="2026-06-14T09:00:00" north="100.0" south="-100.0" east="200.0" west="-200.0"/>
  <road name="Road 1" length="100.0" id="1" junction="-1" rule="RHT">
    <planView><geometry s="0.0" x="0.0" y="0.0" hdg="0.0" length="100.0"><line/></geometry></planView>
    <lanes><laneSection s="0.0"><center><lane id="0" type="none"/></center></laneSection></lanes>
    <tunnel id="t1" s="10.0"/>
  </road>
</OpenDRIVE>)";

  // Act & Assert
  EXPECT_THROW(strada::parser::ParseString(kXml), strada::parser::MissingElementError);
}

TEST_F(BridgesAndTunnelsParserTest, ThrowsInvalidAttributeErrorOnNegativeTunnelS) {
  // Arrange
  const std::string kXml = R"(<?xml version="1.0" standalone="yes"?>
<OpenDRIVE>
  <header revMajor="1" revMinor="9" name="Test Map" version="1.0" date="2026-06-14T09:00:00" north="100.0" south="-100.0" east="200.0" west="-200.0"/>
  <road name="Road 1" length="100.0" id="1" junction="-1" rule="RHT">
    <planView><geometry s="0.0" x="0.0" y="0.0" hdg="0.0" length="100.0"><line/></geometry></planView>
    <lanes><laneSection s="0.0"><center><lane id="0" type="none"/></center></laneSection></lanes>
    <tunnel id="t1" s="-10.0" length="30.0" type="standard"/>
  </road>
</OpenDRIVE>)";

  // Act & Assert
  EXPECT_THROW(strada::parser::ParseString(kXml), strada::parser::InvalidAttributeError);
}

TEST_F(BridgesAndTunnelsParserTest, ThrowsInvalidAttributeErrorOnNegativeTunnelLength) {
  // Arrange
  const std::string kXml = R"(<?xml version="1.0" standalone="yes"?>
<OpenDRIVE>
  <header revMajor="1" revMinor="9" name="Test Map" version="1.0" date="2026-06-14T09:00:00" north="100.0" south="-100.0" east="200.0" west="-200.0"/>
  <road name="Road 1" length="100.0" id="1" junction="-1" rule="RHT">
    <planView><geometry s="0.0" x="0.0" y="0.0" hdg="0.0" length="100.0"><line/></geometry></planView>
    <lanes><laneSection s="0.0"><center><lane id="0" type="none"/></center></laneSection></lanes>
    <tunnel id="t1" s="10.0" length="-30.0" type="standard"/>
  </road>
</OpenDRIVE>)";

  // Act & Assert
  EXPECT_THROW(strada::parser::ParseString(kXml), strada::parser::InvalidAttributeError);
}

TEST_F(BridgesAndTunnelsParserTest, ThrowsMissingElementErrorOnMissingBridgeType) {
  // Arrange
  const std::string kXml = R"(<?xml version="1.0" standalone="yes"?>
<OpenDRIVE>
  <header revMajor="1" revMinor="9" name="Test Map" version="1.0" date="2026-06-14T09:00:00" north="100.0" south="-100.0" east="200.0" west="-200.0"/>
  <road name="Road 1" length="100.0" id="1" junction="-1" rule="RHT">
    <planView><geometry s="0.0" x="0.0" y="0.0" hdg="0.0" length="100.0"><line/></geometry></planView>
    <lanes><laneSection s="0.0"><center><lane id="0" type="none"/></center></laneSection></lanes>
    <bridge id="b1" s="10.0" length="30.0"/>
  </road>
</OpenDRIVE>)";

  // Act & Assert
  EXPECT_THROW(strada::parser::ParseString(kXml), strada::parser::MissingElementError);
}

TEST_F(BridgesAndTunnelsParserTest, ThrowsMissingElementErrorOnMissingTunnelType) {
  // Arrange
  const std::string kXml = R"(<?xml version="1.0" standalone="yes"?>
<OpenDRIVE>
  <header revMajor="1" revMinor="9" name="Test Map" version="1.0" date="2026-06-14T09:00:00" north="100.0" south="-100.0" east="200.0" west="-200.0"/>
  <road name="Road 1" length="100.0" id="1" junction="-1" rule="RHT">
    <planView><geometry s="0.0" x="0.0" y="0.0" hdg="0.0" length="100.0"><line/></geometry></planView>
    <lanes><laneSection s="0.0"><center><lane id="0" type="none"/></center></laneSection></lanes>
    <tunnel id="t1" s="10.0" length="30.0"/>
  </road>
</OpenDRIVE>)";

  // Act & Assert
  EXPECT_THROW(strada::parser::ParseString(kXml), strada::parser::MissingElementError);
}

TEST_F(BridgesAndTunnelsParserTest, ThrowsMissingElementErrorOnMissingLaneValidityFromLane) {
  // Arrange
  const std::string kXml = R"(<?xml version="1.0" standalone="yes"?>
<OpenDRIVE>
  <header revMajor="1" revMinor="9" name="Test Map" version="1.0" date="2026-06-14T09:00:00" north="100.0" south="-100.0" east="200.0" west="-200.0"/>
  <road name="Road 1" length="100.0" id="1" junction="-1" rule="RHT">
    <planView><geometry s="0.0" x="0.0" y="0.0" hdg="0.0" length="100.0"><line/></geometry></planView>
    <lanes><laneSection s="0.0"><center><lane id="0" type="none"/></center></laneSection></lanes>
    <bridge id="b1" s="10.0" length="30.0" type="concrete">
      <validity toLane="-1"/>
    </bridge>
  </road>
</OpenDRIVE>)";

  // Act & Assert
  EXPECT_THROW(strada::parser::ParseString(kXml), strada::parser::MissingElementError);
}

TEST_F(BridgesAndTunnelsParserTest, ThrowsMissingElementErrorOnMissingLaneValidityToLane) {
  // Arrange
  const std::string kXml = R"(<?xml version="1.0" standalone="yes"?>
<OpenDRIVE>
  <header revMajor="1" revMinor="9" name="Test Map" version="1.0" date="2026-06-14T09:00:00" north="100.0" south="-100.0" east="200.0" west="-200.0"/>
  <road name="Road 1" length="100.0" id="1" junction="-1" rule="RHT">
    <planView><geometry s="0.0" x="0.0" y="0.0" hdg="0.0" length="100.0"><line/></geometry></planView>
    <lanes><laneSection s="0.0"><center><lane id="0" type="none"/></center></laneSection></lanes>
    <bridge id="b1" s="10.0" length="30.0" type="concrete">
      <validity fromLane="-2"/>
    </bridge>
  </road>
</OpenDRIVE>)";

  // Act & Assert
  EXPECT_THROW(strada::parser::ParseString(kXml), strada::parser::MissingElementError);
}

TEST_F(BridgesAndTunnelsParserTest, ThrowsInvalidAttributeErrorOnLaneValidityFromLaneGreaterThanToLane) {
  // Arrange
  const std::string kXml = R"(<?xml version="1.0" standalone="yes"?>
<OpenDRIVE>
  <header revMajor="1" revMinor="9" name="Test Map" version="1.0" date="2026-06-14T09:00:00" north="100.0" south="-100.0" east="200.0" west="-200.0"/>
  <road name="Road 1" length="100.0" id="1" junction="-1" rule="RHT">
    <planView><geometry s="0.0" x="0.0" y="0.0" hdg="0.0" length="100.0"><line/></geometry></planView>
    <lanes><laneSection s="0.0"><center><lane id="0" type="none"/></center></laneSection></lanes>
    <bridge id="b1" s="10.0" length="30.0" type="concrete">
      <validity fromLane="-1" toLane="-2"/>
    </bridge>
  </road>
</OpenDRIVE>)";

  // Act & Assert
  EXPECT_THROW(strada::parser::ParseString(kXml), strada::parser::InvalidAttributeError);
}

TEST_F(BridgesAndTunnelsParserTest, ThrowsInvalidAttributeErrorOnInvalidBridgeType) {
  // Arrange
  const std::string kXml = R"(<?xml version="1.0" standalone="yes"?>
<OpenDRIVE>
  <header revMajor="1" revMinor="9" name="Test Map" version="1.0" date="2026-06-14T09:00:00" north="100.0" south="-100.0" east="200.0" west="-200.0"/>
  <road name="Road 1" length="100.0" id="1" junction="-1" rule="RHT">
    <planView><geometry s="0.0" x="0.0" y="0.0" hdg="0.0" length="100.0"><line/></geometry></planView>
    <lanes><laneSection s="0.0"><center><lane id="0" type="none"/></center></laneSection></lanes>
    <bridge id="b1" s="10.0" length="30.0" type="invalid_type"/>
  </road>
</OpenDRIVE>)";

  // Act & Assert
  EXPECT_THROW(strada::parser::ParseString(kXml), strada::parser::InvalidAttributeError);
}

TEST_F(BridgesAndTunnelsParserTest, ThrowsInvalidAttributeErrorOnInvalidTunnelType) {
  // Arrange
  const std::string kXml = R"(<?xml version="1.0" standalone="yes"?>
<OpenDRIVE>
  <header revMajor="1" revMinor="9" name="Test Map" version="1.0" date="2026-06-14T09:00:00" north="100.0" south="-100.0" east="200.0" west="-200.0"/>
  <road name="Road 1" length="100.0" id="1" junction="-1" rule="RHT">
    <planView><geometry s="0.0" x="0.0" y="0.0" hdg="0.0" length="100.0"><line/></geometry></planView>
    <lanes><laneSection s="0.0"><center><lane id="0" type="none"/></center></laneSection></lanes>
    <tunnel id="t1" s="10.0" length="30.0" type="invalid_type"/>
  </road>
</OpenDRIVE>)";

  // Act & Assert
  EXPECT_THROW(strada::parser::ParseString(kXml), strada::parser::InvalidAttributeError);
}

TEST_F(BridgesAndTunnelsParserTest, ThrowsInvalidAttributeErrorOnInvalidLaneValidityLayer) {
  // Arrange
  const std::string kXml = R"(<?xml version="1.0" standalone="yes"?>
<OpenDRIVE>
  <header revMajor="1" revMinor="9" name="Test Map" version="1.0" date="2026-06-14T09:00:00" north="100.0" south="-100.0" east="200.0" west="-200.0"/>
  <road name="Road 1" length="100.0" id="1" junction="-1" rule="RHT">
    <planView><geometry s="0.0" x="0.0" y="0.0" hdg="0.0" length="100.0"><line/></geometry></planView>
    <lanes><laneSection s="0.0"><center><lane id="0" type="none"/></center></laneSection></lanes>
    <bridge id="b1" s="10.0" length="30.0" type="concrete">
      <validity fromLane="-2" toLane="-1" layer="invalid_layer"/>
    </bridge>
  </road>
</OpenDRIVE>)";

  // Act & Assert
  EXPECT_THROW(strada::parser::ParseString(kXml), strada::parser::InvalidAttributeError);
}

TEST(ParserTest, ThrowsInvalidAttributeErrorOnInvalidTrafficRule) {
  // Arrange
  const std::string kXml = R"(<?xml version="1.0" standalone="yes"?>
<OpenDRIVE>
  <header revMajor="1" revMinor="9" name="Test Map" version="1.0" date="2026-06-14T09:00:00" north="100.0" south="-100.0" east="200.0" west="-200.0"/>
  <road name="Road 1" length="100.0" id="1" junction="-1" rule="INVALID">
    <planView><geometry s="0.0" x="0.0" y="0.0" hdg="0.0" length="100.0"><line/></geometry></planView>
  </road>
</OpenDRIVE>)";

  // Act & Assert
  EXPECT_THROW(strada::parser::ParseString(kXml), strada::parser::InvalidAttributeError);
}

TEST(ParserTest, DefaultTrafficRuleWhenMissing) {
  // Arrange
  const std::string kXml = R"(<?xml version="1.0" standalone="yes"?>
<OpenDRIVE>
  <header revMajor="1" revMinor="9" name="Test Map" version="1.0" date="2026-06-14T09:00:00" north="100.0" south="-100.0" east="200.0" west="-200.0"/>
  <road name="Road 1" length="100.0" id="1" junction="-1">
    <planView><geometry s="0.0" x="0.0" y="0.0" hdg="0.0" length="100.0"><line/></geometry></planView>
  </road>
</OpenDRIVE>)";

  // Act
  auto ast_tree = strada::parser::ParseString(kXml);

  // Assert
  ASSERT_EQ(ast_tree.roads.size(), 1);
  EXPECT_EQ(ast_tree.roads[0].rule, strada::ast::TrafficRule::kRht);
}

TEST(ParserTest, MissingRoadNameIsNullopt) {
  // Arrange
  const std::string kXml = R"(<?xml version="1.0" standalone="yes"?>
<OpenDRIVE>
  <header revMajor="1" revMinor="9" name="Test Map" version="1.0" date="2026-06-14T09:00:00" north="100.0" south="-100.0" east="200.0" west="-200.0"/>
  <road length="100.0" id="1" junction="-1">
    <planView><geometry s="0.0" x="0.0" y="0.0" hdg="0.0" length="100.0"><line/></geometry></planView>
  </road>
</OpenDRIVE>)";

  // Act
  auto ast_tree = strada::parser::ParseString(kXml);

  // Assert
  ASSERT_EQ(ast_tree.roads.size(), 1);
  EXPECT_EQ(ast_tree.roads[0].name, std::nullopt);
}

TEST(ParserTest, ParseLaneTypesToEnum) {
  // Arrange
  const std::string kXml = R"(<?xml version="1.0" standalone="yes"?>
<OpenDRIVE>
  <header revMajor="1" revMinor="9" name="Test Map" version="1.0" date="2026-06-14T09:00:00" north="100.0" south="-100.0" east="200.0" west="-200.0"/>
  <road name="Road 1" length="100.0" id="1" junction="-1">
    <planView><geometry s="0.0" x="0.0" y="0.0" hdg="0.0" length="100.0"><line/></geometry></planView>
    <lanes>
      <laneSection s="0.0">
        <center>
          <lane id="0" type="none"/>
        </center>
        <left>
          <lane id="1" type="driving"/>
          <lane id="2" type="hov"/>
        </left>
        <right>
          <lane id="-1" type="sidewalk"/>
          <lane id="-2" type="shoulder"/>
        </right>
      </laneSection>
    </lanes>
  </road>
</OpenDRIVE>)";

  // Act
  auto ast_tree = strada::parser::ParseString(kXml);

  // Assert
  ASSERT_EQ(ast_tree.roads.size(), 1);
  const auto& road = ast_tree.roads[0];
  ASSERT_EQ(road.lanes.sections.size(), 1);
  const auto& section = road.lanes.sections[0];
  EXPECT_EQ(section.center[0].type, strada::ast::LaneType::kNone);
  EXPECT_EQ(section.left[0].type, strada::ast::LaneType::kDriving);
  EXPECT_EQ(section.left[1].type, strada::ast::LaneType::kHov);
  EXPECT_EQ(section.right[0].type, strada::ast::LaneType::kSidewalk);
  EXPECT_EQ(section.right[1].type, strada::ast::LaneType::kShoulder);
}

TEST(ParserTest, ThrowsMissingElementErrorOnMissingLaneType) {
  // Arrange
  const std::string kXml = R"(<?xml version="1.0" standalone="yes"?>
<OpenDRIVE>
  <header revMajor="1" revMinor="9" name="Test Map" version="1.0" date="2026-06-14T09:00:00" north="100.0" south="-100.0" east="200.0" west="-200.0"/>
  <road name="Road 1" length="100.0" id="1" junction="-1">
    <planView><geometry s="0.0" x="0.0" y="0.0" hdg="0.0" length="100.0"><line/></geometry></planView>
    <lanes>
      <laneSection s="0.0">
        <center>
          <lane id="0"/>
        </center>
      </laneSection>
    </lanes>
  </road>
</OpenDRIVE>)";

  // Act & Assert
  EXPECT_THROW(strada::parser::ParseString(kXml), strada::parser::MissingElementError);
}

TEST(ParserTest, ThrowsInvalidAttributeErrorOnInvalidLaneType) {
  // Arrange
  const std::string kXml = R"(<?xml version="1.0" standalone="yes"?>
<OpenDRIVE>
  <header revMajor="1" revMinor="9" name="Test Map" version="1.0" date="2026-06-14T09:00:00" north="100.0" south="-100.0" east="200.0" west="-200.0"/>
  <road name="Road 1" length="100.0" id="1" junction="-1">
    <planView><geometry s="0.0" x="0.0" y="0.0" hdg="0.0" length="100.0"><line/></geometry></planView>
    <lanes>
      <laneSection s="0.0">
        <center>
          <lane id="0" type="invalid_type"/>
        </center>
      </laneSection>
    </lanes>
  </road>
</OpenDRIVE>)";

  // Act & Assert
  EXPECT_THROW(strada::parser::ParseString(kXml), strada::parser::InvalidAttributeError);
}

TEST(ParserTest, ThrowsMissingElementErrorOnMissingPredecessorId) {
  // Arrange
  const std::string kXml = R"(<?xml version="1.0" standalone="yes"?>
<OpenDRIVE>
  <header revMajor="1" revMinor="9" name="Test Map" version="1.0" date="2026-06-14T09:00:00" north="100.0" south="-100.0" east="200.0" west="-200.0"/>
  <road name="Road 1" length="100.0" id="1" junction="-1">
    <planView><geometry s="0.0" x="0.0" y="0.0" hdg="0.0" length="100.0"><line/></geometry></planView>
    <lanes>
      <laneSection s="0.0">
        <center>
          <lane id="0" type="none"/>
        </center>
        <left>
          <lane id="1" type="driving">
            <link>
              <predecessor/>
            </link>
          </lane>
        </left>
      </laneSection>
    </lanes>
  </road>
</OpenDRIVE>)";

  // Act & Assert
  EXPECT_THROW(strada::parser::ParseString(kXml), strada::parser::MissingElementError);
}

TEST(ParserTest, ThrowsMissingElementErrorOnMissingSuccessorId) {
  // Arrange
  const std::string kXml = R"(<?xml version="1.0" standalone="yes"?>
<OpenDRIVE>
  <header revMajor="1" revMinor="9" name="Test Map" version="1.0" date="2026-06-14T09:00:00" north="100.0" south="-100.0" east="200.0" west="-200.0"/>
  <road name="Road 1" length="100.0" id="1" junction="-1">
    <planView><geometry s="0.0" x="0.0" y="0.0" hdg="0.0" length="100.0"><line/></geometry></planView>
    <lanes>
      <laneSection s="0.0">
        <center>
          <lane id="0" type="none"/>
        </center>
        <left>
          <lane id="1" type="driving">
            <link>
              <successor/>
            </link>
          </lane>
        </left>
      </laneSection>
    </lanes>
  </road>
</OpenDRIVE>)";

  // Act & Assert
  EXPECT_THROW(strada::parser::ParseString(kXml), strada::parser::MissingElementError);
}

TEST(ParserTest, DefaultObjectFields) {
  // Arrange & Act
  strada::ast::Object obj;
  strada::ast::ObjectReference obj_ref;

  // Assert
  EXPECT_EQ(obj.type, strada::ast::ObjectType::kNone);
  EXPECT_EQ(obj.orientation, strada::ast::Orientation::kNone);
  EXPECT_EQ(static_cast<std::uint8_t>(obj.orientation), 0);

  EXPECT_EQ(obj_ref.orientation, strada::ast::Orientation::kNone);
  EXPECT_EQ(static_cast<std::uint8_t>(obj_ref.orientation), 0);
}

TEST(ParserConversionsTest, TrafficRuleConversions) {
  // Arrange & Act & Assert
  EXPECT_EQ(strada::parser::FromString<strada::ast::TrafficRule>("RHT"), strada::ast::TrafficRule::kRht);
  EXPECT_EQ(strada::parser::FromString<strada::ast::TrafficRule>("LHT"), strada::ast::TrafficRule::kLht);
  EXPECT_EQ(strada::parser::FromString<strada::ast::TrafficRule>("invalid"), std::nullopt);

  EXPECT_EQ(strada::parser::ToString(strada::ast::TrafficRule::kRht), "RHT");
  EXPECT_EQ(strada::parser::ToString(strada::ast::TrafficRule::kLht), "LHT");
}

TEST(ParserConversionsTest, OrientationConversions) {
  // Arrange & Act & Assert
  EXPECT_EQ(strada::parser::FromString<strada::ast::Orientation>("none"), strada::ast::Orientation::kNone);
  EXPECT_EQ(strada::parser::FromString<strada::ast::Orientation>("+"), strada::ast::Orientation::kPlus);
  EXPECT_EQ(strada::parser::FromString<strada::ast::Orientation>("-"), strada::ast::Orientation::kMinus);
  EXPECT_EQ(strada::parser::FromString<strada::ast::Orientation>("invalid"), std::nullopt);

  EXPECT_EQ(strada::parser::ToString(strada::ast::Orientation::kNone), "none");
  EXPECT_EQ(strada::parser::ToString(strada::ast::Orientation::kPlus), "+");
  EXPECT_EQ(strada::parser::ToString(strada::ast::Orientation::kMinus), "-");
}

TEST(ParserConversionsTest, PRangeConversions) {
  // Arrange & Act & Assert
  EXPECT_EQ(strada::parser::FromString<strada::ast::PRange>("normalized"), strada::ast::PRange::kNormalized);
  EXPECT_EQ(strada::parser::FromString<strada::ast::PRange>("arcLength"), strada::ast::PRange::kArcLength);
  EXPECT_EQ(strada::parser::FromString<strada::ast::PRange>("invalid"), std::nullopt);

  EXPECT_EQ(strada::parser::ToString(strada::ast::PRange::kNormalized), "normalized");
  EXPECT_EQ(strada::parser::ToString(strada::ast::PRange::kArcLength), "arcLength");
}

TEST(ParserConversionsTest, ContactPointConversions) {
  // Arrange & Act & Assert
  EXPECT_EQ(strada::parser::FromString<strada::ast::ContactPoint>("start"), strada::ast::ContactPoint::kStart);
  EXPECT_EQ(strada::parser::FromString<strada::ast::ContactPoint>("end"), strada::ast::ContactPoint::kEnd);
  EXPECT_EQ(strada::parser::FromString<strada::ast::ContactPoint>("invalid"), std::nullopt);

  EXPECT_EQ(strada::parser::ToString(strada::ast::ContactPoint::kStart), "start");
  EXPECT_EQ(strada::parser::ToString(strada::ast::ContactPoint::kEnd), "end");
}

TEST(ParserConversionsTest, JunctionSegmentTypeConversions) {
  // Arrange & Act & Assert
  EXPECT_EQ(strada::parser::FromString<strada::ast::JunctionSegmentType>("lane"),
            strada::ast::JunctionSegmentType::kLane);
  EXPECT_EQ(strada::parser::FromString<strada::ast::JunctionSegmentType>("joint"),
            strada::ast::JunctionSegmentType::kJoint);
  EXPECT_EQ(strada::parser::FromString<strada::ast::JunctionSegmentType>("invalid"), std::nullopt);

  EXPECT_EQ(strada::parser::ToString(strada::ast::JunctionSegmentType::kLane), "lane");
  EXPECT_EQ(strada::parser::ToString(strada::ast::JunctionSegmentType::kJoint), "joint");
}

TEST(ParserConversionsTest, StripModeConversions) {
  // Arrange & Act & Assert
  EXPECT_EQ(strada::parser::FromString<strada::ast::StripMode>("independent"), strada::ast::StripMode::kIndependent);
  EXPECT_EQ(strada::parser::FromString<strada::ast::StripMode>("relative"), strada::ast::StripMode::kRelative);
  EXPECT_EQ(strada::parser::FromString<strada::ast::StripMode>("invalid"), std::nullopt);

  EXPECT_EQ(strada::parser::ToString(strada::ast::StripMode::kIndependent), "independent");
  EXPECT_EQ(strada::parser::ToString(strada::ast::StripMode::kRelative), "relative");
}

TEST(ParserTest, ThrowsInvalidAttributeErrorOnInvalidPRange) {
  // Arrange
  const std::string kXml = R"(<?xml version="1.0" standalone="yes"?>
<OpenDRIVE>
  <header revMajor="1" revMinor="9" name="Test Map" version="1.0" date="2026-06-14T09:00:00" north="100.0" south="-100.0" east="200.0" west="-200.0"/>
  <road name="Road 1" length="100.0" id="1" junction="-1">
    <planView>
      <geometry s="0.0" x="0.0" y="0.0" hdg="0.0" length="100.0">
        <paramPoly3 aU="0.0" bU="0.0" cU="0.0" dU="0.0" aV="0.0" bV="0.0" cV="0.0" dV="0.0" pRange="INVALID"/>
      </geometry>
    </planView>
  </road>
</OpenDRIVE>)";

  // Act & Assert
  EXPECT_THROW(strada::parser::ParseString(kXml), strada::parser::InvalidAttributeError);
}

TEST(ParserTest, ThrowsInvalidAttributeErrorOnInvalidJunctionSegmentType) {
  // Arrange
  const std::string kXml = R"(<?xml version="1.0" standalone="yes"?>
<OpenDRIVE>
  <header revMajor="1" revMinor="9" name="Test Map" version="1.0" date="2026-06-14T09:00:00" north="100.0" south="-100.0" east="200.0" west="-200.0"/>
  <junction id="1" name="Junction 1">
    <connection id="0" incomingRoad="1" connectingRoad="2" contactPoint="start"/>
    <boundary>
      <segment type="INVALID" roadId="1"/>
    </boundary>
  </junction>
</OpenDRIVE>)";

  // Act & Assert
  EXPECT_THROW(strada::parser::ParseString(kXml), strada::parser::InvalidAttributeError);
}

TEST(ParserTest, ThrowsInvalidAttributeErrorOnInvalidJunctionBoundaryContactPoint) {
  // Arrange
  const std::string kXml = R"(<?xml version="1.0" standalone="yes"?>
<OpenDRIVE>
  <header revMajor="1" revMinor="9" name="Test Map" version="1.0" date="2026-06-14T09:00:00" north="100.0" south="-100.0" east="200.0" west="-200.0"/>
  <junction id="1" name="Junction 1">
    <connection id="0" incomingRoad="1" connectingRoad="2" contactPoint="start"/>
    <boundary>
      <segment type="joint" roadId="1" contactPoint="INVALID"/>
    </boundary>
  </junction>
</OpenDRIVE>)";

  // Act & Assert
  EXPECT_THROW(strada::parser::ParseString(kXml), strada::parser::InvalidAttributeError);
}

TEST(ParserTest, ThrowsInvalidAttributeErrorOnInvalidJunctionConnectionContactPoint) {
  // Arrange
  const std::string kXml = R"(<?xml version="1.0" standalone="yes"?>
<OpenDRIVE>
  <header revMajor="1" revMinor="9" name="Test Map" version="1.0" date="2026-06-14T09:00:00" north="100.0" south="-100.0" east="200.0" west="-200.0"/>
  <junction id="1" name="Junction 1">
    <connection id="0" incomingRoad="1" connectingRoad="2" contactPoint="INVALID"/>
  </junction>
</OpenDRIVE>)";

  // Act & Assert
  EXPECT_THROW(strada::parser::ParseString(kXml), strada::parser::InvalidAttributeError);
}
