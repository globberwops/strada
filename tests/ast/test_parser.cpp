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

TEST(ParserTest, ParsesLaneMissingTypeAsNone) {
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

  // Act
  auto map = strada::parser::ParseString(kXml);

  // Assert
  ASSERT_EQ(map.roads.size(), 1);
  ASSERT_EQ(map.roads[0].lanes.sections.size(), 1);
  ASSERT_EQ(map.roads[0].lanes.sections[0].center.size(), 1);
  EXPECT_EQ(map.roads[0].lanes.sections[0].center[0].type, strada::ast::LaneType::kNone);
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

TEST(ParserConversionsTest, JunctionTypeConversions) {
  // Arrange & Act & Assert
  EXPECT_EQ(strada::parser::FromString<strada::ast::JunctionType>("default"), strada::ast::JunctionType::kCommon);
  EXPECT_EQ(strada::parser::FromString<strada::ast::JunctionType>("crossing"), strada::ast::JunctionType::kCrossing);
  EXPECT_EQ(strada::parser::FromString<strada::ast::JunctionType>("direct"), strada::ast::JunctionType::kDirect);
  EXPECT_EQ(strada::parser::FromString<strada::ast::JunctionType>("virtual"), strada::ast::JunctionType::kVirtual);
  EXPECT_EQ(strada::parser::FromString<strada::ast::JunctionType>("invalid"), std::nullopt);

  EXPECT_EQ(strada::parser::ToString(strada::ast::JunctionType::kCommon), "default");
  EXPECT_EQ(strada::parser::ToString(strada::ast::JunctionType::kCrossing), "crossing");
  EXPECT_EQ(strada::parser::ToString(strada::ast::JunctionType::kDirect), "direct");
  EXPECT_EQ(strada::parser::ToString(strada::ast::JunctionType::kVirtual), "virtual");
}

TEST(ParserConversionsTest, LayerTypeConversions) {
  // Arrange & Act & Assert
  EXPECT_EQ(strada::parser::FromString<strada::ast::LayerType>("permanent"), strada::ast::LayerType::kPermanent);
  EXPECT_EQ(strada::parser::FromString<strada::ast::LayerType>("temporary"), strada::ast::LayerType::kTemporary);
  EXPECT_EQ(strada::parser::FromString<strada::ast::LayerType>("invalid"), std::nullopt);

  EXPECT_EQ(strada::parser::ToString(strada::ast::LayerType::kPermanent), "permanent");
  EXPECT_EQ(strada::parser::ToString(strada::ast::LayerType::kTemporary), "temporary");
}

TEST(ParserConversionsTest, TunnelTypeConversions) {
  // Arrange & Act & Assert
  EXPECT_EQ(strada::parser::FromString<strada::ast::TunnelType>("standard"), strada::ast::TunnelType::kStandard);
  EXPECT_EQ(strada::parser::FromString<strada::ast::TunnelType>("underpass"), strada::ast::TunnelType::kUnderpass);
  EXPECT_EQ(strada::parser::FromString<strada::ast::TunnelType>("invalid"), std::nullopt);

  EXPECT_EQ(strada::parser::ToString(strada::ast::TunnelType::kStandard), "standard");
  EXPECT_EQ(strada::parser::ToString(strada::ast::TunnelType::kUnderpass), "underpass");
}

TEST(ParserConversionsTest, BridgeTypeConversions) {
  // Arrange & Act & Assert
  EXPECT_EQ(strada::parser::FromString<strada::ast::BridgeType>("brick"), strada::ast::BridgeType::kBrick);
  EXPECT_EQ(strada::parser::FromString<strada::ast::BridgeType>("concrete"), strada::ast::BridgeType::kConcrete);
  EXPECT_EQ(strada::parser::FromString<strada::ast::BridgeType>("steel"), strada::ast::BridgeType::kSteel);
  EXPECT_EQ(strada::parser::FromString<strada::ast::BridgeType>("wood"), strada::ast::BridgeType::kWood);
  EXPECT_EQ(strada::parser::FromString<strada::ast::BridgeType>("invalid"), std::nullopt);

  EXPECT_EQ(strada::parser::ToString(strada::ast::BridgeType::kBrick), "brick");
  EXPECT_EQ(strada::parser::ToString(strada::ast::BridgeType::kConcrete), "concrete");
  EXPECT_EQ(strada::parser::ToString(strada::ast::BridgeType::kSteel), "steel");
  EXPECT_EQ(strada::parser::ToString(strada::ast::BridgeType::kWood), "wood");
}

TEST(ParserConversionsTest, LaneTypeConversions) {
  // Arrange & Act & Assert
  EXPECT_EQ(strada::parser::FromString<strada::ast::LaneType>("hov"), strada::ast::LaneType::kHov);
  EXPECT_EQ(strada::parser::FromString<strada::ast::LaneType>("bidirectional"), strada::ast::LaneType::kBidirectional);
  EXPECT_EQ(strada::parser::FromString<strada::ast::LaneType>("biking"), strada::ast::LaneType::kBiking);
  EXPECT_EQ(strada::parser::FromString<strada::ast::LaneType>("border"), strada::ast::LaneType::kBorder);
  EXPECT_EQ(strada::parser::FromString<strada::ast::LaneType>("bus"), strada::ast::LaneType::kBus);
  EXPECT_EQ(strada::parser::FromString<strada::ast::LaneType>("connectingRamp"),
            strada::ast::LaneType::kConnectingRamp);
  EXPECT_EQ(strada::parser::FromString<strada::ast::LaneType>("curb"), strada::ast::LaneType::kCurb);
  EXPECT_EQ(strada::parser::FromString<strada::ast::LaneType>("driving"), strada::ast::LaneType::kDriving);
  EXPECT_EQ(strada::parser::FromString<strada::ast::LaneType>("entry"), strada::ast::LaneType::kEntry);
  EXPECT_EQ(strada::parser::FromString<strada::ast::LaneType>("exit"), strada::ast::LaneType::kExit);
  EXPECT_EQ(strada::parser::FromString<strada::ast::LaneType>("median"), strada::ast::LaneType::kMedian);
  EXPECT_EQ(strada::parser::FromString<strada::ast::LaneType>("mwyEntry"), strada::ast::LaneType::kMwyEntry);
  EXPECT_EQ(strada::parser::FromString<strada::ast::LaneType>("mwyExit"), strada::ast::LaneType::kMwyExit);
  EXPECT_EQ(strada::parser::FromString<strada::ast::LaneType>("none"), strada::ast::LaneType::kNone);
  EXPECT_EQ(strada::parser::FromString<strada::ast::LaneType>("offRamp"), strada::ast::LaneType::kOffRamp);
  EXPECT_EQ(strada::parser::FromString<strada::ast::LaneType>("onRamp"), strada::ast::LaneType::kOnRamp);
  EXPECT_EQ(strada::parser::FromString<strada::ast::LaneType>("parking"), strada::ast::LaneType::kParking);
  EXPECT_EQ(strada::parser::FromString<strada::ast::LaneType>("rail"), strada::ast::LaneType::kRail);
  EXPECT_EQ(strada::parser::FromString<strada::ast::LaneType>("restricted"), strada::ast::LaneType::kRestricted);
  EXPECT_EQ(strada::parser::FromString<strada::ast::LaneType>("roadWorks"), strada::ast::LaneType::kRoadWorks);
  EXPECT_EQ(strada::parser::FromString<strada::ast::LaneType>("shared"), strada::ast::LaneType::kShared);
  EXPECT_EQ(strada::parser::FromString<strada::ast::LaneType>("shoulder"), strada::ast::LaneType::kShoulder);
  EXPECT_EQ(strada::parser::FromString<strada::ast::LaneType>("sidewalk"), strada::ast::LaneType::kSidewalk);
  EXPECT_EQ(strada::parser::FromString<strada::ast::LaneType>("slipLane"), strada::ast::LaneType::kSlipLane);
  EXPECT_EQ(strada::parser::FromString<strada::ast::LaneType>("special1"), strada::ast::LaneType::kSpecial1);
  EXPECT_EQ(strada::parser::FromString<strada::ast::LaneType>("special2"), strada::ast::LaneType::kSpecial2);
  EXPECT_EQ(strada::parser::FromString<strada::ast::LaneType>("special3"), strada::ast::LaneType::kSpecial3);
  EXPECT_EQ(strada::parser::FromString<strada::ast::LaneType>("stop"), strada::ast::LaneType::kStop);
  EXPECT_EQ(strada::parser::FromString<strada::ast::LaneType>("taxi"), strada::ast::LaneType::kTaxi);
  EXPECT_EQ(strada::parser::FromString<strada::ast::LaneType>("tram"), strada::ast::LaneType::kTram);
  EXPECT_EQ(strada::parser::FromString<strada::ast::LaneType>("walking"), strada::ast::LaneType::kWalking);
  EXPECT_EQ(strada::parser::FromString<strada::ast::LaneType>("invalid"), std::nullopt);

  EXPECT_EQ(strada::parser::ToString(strada::ast::LaneType::kHov), "hov");
  EXPECT_EQ(strada::parser::ToString(strada::ast::LaneType::kBidirectional), "bidirectional");
  EXPECT_EQ(strada::parser::ToString(strada::ast::LaneType::kBiking), "biking");
  EXPECT_EQ(strada::parser::ToString(strada::ast::LaneType::kBorder), "border");
  EXPECT_EQ(strada::parser::ToString(strada::ast::LaneType::kBus), "bus");
  EXPECT_EQ(strada::parser::ToString(strada::ast::LaneType::kConnectingRamp), "connectingRamp");
  EXPECT_EQ(strada::parser::ToString(strada::ast::LaneType::kCurb), "curb");
  EXPECT_EQ(strada::parser::ToString(strada::ast::LaneType::kDriving), "driving");
  EXPECT_EQ(strada::parser::ToString(strada::ast::LaneType::kEntry), "entry");
  EXPECT_EQ(strada::parser::ToString(strada::ast::LaneType::kExit), "exit");
  EXPECT_EQ(strada::parser::ToString(strada::ast::LaneType::kMedian), "median");
  EXPECT_EQ(strada::parser::ToString(strada::ast::LaneType::kMwyEntry), "mwyEntry");
  EXPECT_EQ(strada::parser::ToString(strada::ast::LaneType::kMwyExit), "mwyExit");
  EXPECT_EQ(strada::parser::ToString(strada::ast::LaneType::kNone), "none");
  EXPECT_EQ(strada::parser::ToString(strada::ast::LaneType::kOffRamp), "offRamp");
  EXPECT_EQ(strada::parser::ToString(strada::ast::LaneType::kOnRamp), "onRamp");
  EXPECT_EQ(strada::parser::ToString(strada::ast::LaneType::kParking), "parking");
  EXPECT_EQ(strada::parser::ToString(strada::ast::LaneType::kRail), "rail");
  EXPECT_EQ(strada::parser::ToString(strada::ast::LaneType::kRestricted), "restricted");
  EXPECT_EQ(strada::parser::ToString(strada::ast::LaneType::kRoadWorks), "roadWorks");
  EXPECT_EQ(strada::parser::ToString(strada::ast::LaneType::kShared), "shared");
  EXPECT_EQ(strada::parser::ToString(strada::ast::LaneType::kShoulder), "shoulder");
  EXPECT_EQ(strada::parser::ToString(strada::ast::LaneType::kSidewalk), "sidewalk");
  EXPECT_EQ(strada::parser::ToString(strada::ast::LaneType::kSlipLane), "slipLane");
  EXPECT_EQ(strada::parser::ToString(strada::ast::LaneType::kSpecial1), "special1");
  EXPECT_EQ(strada::parser::ToString(strada::ast::LaneType::kSpecial2), "special2");
  EXPECT_EQ(strada::parser::ToString(strada::ast::LaneType::kSpecial3), "special3");
  EXPECT_EQ(strada::parser::ToString(strada::ast::LaneType::kStop), "stop");
  EXPECT_EQ(strada::parser::ToString(strada::ast::LaneType::kTaxi), "taxi");
  EXPECT_EQ(strada::parser::ToString(strada::ast::LaneType::kTram), "tram");
  EXPECT_EQ(strada::parser::ToString(strada::ast::LaneType::kWalking), "walking");
}

TEST(ParserConversionsTest, ObjectTypeConversions) {
  // Arrange & Act & Assert
  EXPECT_EQ(strada::parser::FromString<strada::ast::ObjectType>("none"), strada::ast::ObjectType::kNone);
  EXPECT_EQ(strada::parser::FromString<strada::ast::ObjectType>("obstacle"), strada::ast::ObjectType::kObstacle);
  EXPECT_EQ(strada::parser::FromString<strada::ast::ObjectType>("car"), strada::ast::ObjectType::kCar);
  EXPECT_EQ(strada::parser::FromString<strada::ast::ObjectType>("pole"), strada::ast::ObjectType::kPole);
  EXPECT_EQ(strada::parser::FromString<strada::ast::ObjectType>("tree"), strada::ast::ObjectType::kTree);
  EXPECT_EQ(strada::parser::FromString<strada::ast::ObjectType>("vegetation"), strada::ast::ObjectType::kVegetation);
  EXPECT_EQ(strada::parser::FromString<strada::ast::ObjectType>("barrier"), strada::ast::ObjectType::kBarrier);
  EXPECT_EQ(strada::parser::FromString<strada::ast::ObjectType>("building"), strada::ast::ObjectType::kBuilding);
  EXPECT_EQ(strada::parser::FromString<strada::ast::ObjectType>("parkingSpace"),
            strada::ast::ObjectType::kParkingSpace);
  EXPECT_EQ(strada::parser::FromString<strada::ast::ObjectType>("patch"), strada::ast::ObjectType::kPatch);
  EXPECT_EQ(strada::parser::FromString<strada::ast::ObjectType>("railing"), strada::ast::ObjectType::kRailing);
  EXPECT_EQ(strada::parser::FromString<strada::ast::ObjectType>("trafficIsland"),
            strada::ast::ObjectType::kTrafficIsland);
  EXPECT_EQ(strada::parser::FromString<strada::ast::ObjectType>("crosswalk"), strada::ast::ObjectType::kCrosswalk);
  EXPECT_EQ(strada::parser::FromString<strada::ast::ObjectType>("streetLamp"), strada::ast::ObjectType::kStreetLamp);
  EXPECT_EQ(strada::parser::FromString<strada::ast::ObjectType>("gantry"), strada::ast::ObjectType::kGantry);
  EXPECT_EQ(strada::parser::FromString<strada::ast::ObjectType>("soundBarrier"),
            strada::ast::ObjectType::kSoundBarrier);
  EXPECT_EQ(strada::parser::FromString<strada::ast::ObjectType>("van"), strada::ast::ObjectType::kVan);
  EXPECT_EQ(strada::parser::FromString<strada::ast::ObjectType>("bus"), strada::ast::ObjectType::kBus);
  EXPECT_EQ(strada::parser::FromString<strada::ast::ObjectType>("trailer"), strada::ast::ObjectType::kTrailer);
  EXPECT_EQ(strada::parser::FromString<strada::ast::ObjectType>("bike"), strada::ast::ObjectType::kBike);
  EXPECT_EQ(strada::parser::FromString<strada::ast::ObjectType>("motorbike"), strada::ast::ObjectType::kMotorbike);
  EXPECT_EQ(strada::parser::FromString<strada::ast::ObjectType>("tram"), strada::ast::ObjectType::kTram);
  EXPECT_EQ(strada::parser::FromString<strada::ast::ObjectType>("train"), strada::ast::ObjectType::kTrain);
  EXPECT_EQ(strada::parser::FromString<strada::ast::ObjectType>("pedestrian"), strada::ast::ObjectType::kPedestrian);
  EXPECT_EQ(strada::parser::FromString<strada::ast::ObjectType>("wind"), strada::ast::ObjectType::kWind);
  EXPECT_EQ(strada::parser::FromString<strada::ast::ObjectType>("roadMark"), strada::ast::ObjectType::kRoadMark);
  EXPECT_EQ(strada::parser::FromString<strada::ast::ObjectType>("roadSurface"), strada::ast::ObjectType::kRoadSurface);
  EXPECT_EQ(strada::parser::FromString<strada::ast::ObjectType>("invalid"), std::nullopt);

  EXPECT_EQ(strada::parser::ToString(strada::ast::ObjectType::kNone), "none");
  EXPECT_EQ(strada::parser::ToString(strada::ast::ObjectType::kObstacle), "obstacle");
  EXPECT_EQ(strada::parser::ToString(strada::ast::ObjectType::kCar), "car");
  EXPECT_EQ(strada::parser::ToString(strada::ast::ObjectType::kPole), "pole");
  EXPECT_EQ(strada::parser::ToString(strada::ast::ObjectType::kTree), "tree");
  EXPECT_EQ(strada::parser::ToString(strada::ast::ObjectType::kVegetation), "vegetation");
  EXPECT_EQ(strada::parser::ToString(strada::ast::ObjectType::kBarrier), "barrier");
  EXPECT_EQ(strada::parser::ToString(strada::ast::ObjectType::kBuilding), "building");
  EXPECT_EQ(strada::parser::ToString(strada::ast::ObjectType::kParkingSpace), "parkingSpace");
  EXPECT_EQ(strada::parser::ToString(strada::ast::ObjectType::kPatch), "patch");
  EXPECT_EQ(strada::parser::ToString(strada::ast::ObjectType::kRailing), "railing");
  EXPECT_EQ(strada::parser::ToString(strada::ast::ObjectType::kTrafficIsland), "trafficIsland");
  EXPECT_EQ(strada::parser::ToString(strada::ast::ObjectType::kCrosswalk), "crosswalk");
  EXPECT_EQ(strada::parser::ToString(strada::ast::ObjectType::kStreetLamp), "streetLamp");
  EXPECT_EQ(strada::parser::ToString(strada::ast::ObjectType::kGantry), "gantry");
  EXPECT_EQ(strada::parser::ToString(strada::ast::ObjectType::kSoundBarrier), "soundBarrier");
  EXPECT_EQ(strada::parser::ToString(strada::ast::ObjectType::kVan), "van");
  EXPECT_EQ(strada::parser::ToString(strada::ast::ObjectType::kBus), "bus");
  EXPECT_EQ(strada::parser::ToString(strada::ast::ObjectType::kTrailer), "trailer");
  EXPECT_EQ(strada::parser::ToString(strada::ast::ObjectType::kBike), "bike");
  EXPECT_EQ(strada::parser::ToString(strada::ast::ObjectType::kMotorbike), "motorbike");
  EXPECT_EQ(strada::parser::ToString(strada::ast::ObjectType::kTram), "tram");
  EXPECT_EQ(strada::parser::ToString(strada::ast::ObjectType::kTrain), "train");
  EXPECT_EQ(strada::parser::ToString(strada::ast::ObjectType::kPedestrian), "pedestrian");
  EXPECT_EQ(strada::parser::ToString(strada::ast::ObjectType::kWind), "wind");
  EXPECT_EQ(strada::parser::ToString(strada::ast::ObjectType::kRoadMark), "roadMark");
  EXPECT_EQ(strada::parser::ToString(strada::ast::ObjectType::kRoadSurface), "roadSurface");
}

TEST(ParserConversionsTest, RoadTypeConversions) {
  // Arrange & Act & Assert
  EXPECT_EQ(strada::parser::FromString<strada::ast::RoadType>("unknown"), strada::ast::RoadType::kUnknown);
  EXPECT_EQ(strada::parser::FromString<strada::ast::RoadType>("bicycle"), strada::ast::RoadType::kBicycle);
  EXPECT_EQ(strada::parser::FromString<strada::ast::RoadType>("lowSpeed"), strada::ast::RoadType::kLowSpeed);
  EXPECT_EQ(strada::parser::FromString<strada::ast::RoadType>("pedestrian"), strada::ast::RoadType::kPedestrian);
  EXPECT_EQ(strada::parser::FromString<strada::ast::RoadType>("motorway"), strada::ast::RoadType::kMotorway);
  EXPECT_EQ(strada::parser::FromString<strada::ast::RoadType>("rural"), strada::ast::RoadType::kRural);
  EXPECT_EQ(strada::parser::FromString<strada::ast::RoadType>("townArterial"), strada::ast::RoadType::kTownArterial);
  EXPECT_EQ(strada::parser::FromString<strada::ast::RoadType>("townCollector"), strada::ast::RoadType::kTownCollector);
  EXPECT_EQ(strada::parser::FromString<strada::ast::RoadType>("townExpressway"),
            strada::ast::RoadType::kTownExpressway);
  EXPECT_EQ(strada::parser::FromString<strada::ast::RoadType>("townLocal"), strada::ast::RoadType::kTownLocal);
  EXPECT_EQ(strada::parser::FromString<strada::ast::RoadType>("townPlayStreet"),
            strada::ast::RoadType::kTownPlayStreet);
  EXPECT_EQ(strada::parser::FromString<strada::ast::RoadType>("townPrivate"), strada::ast::RoadType::kTownPrivate);
  EXPECT_EQ(strada::parser::FromString<strada::ast::RoadType>("town"), strada::ast::RoadType::kTown);
  EXPECT_EQ(strada::parser::FromString<strada::ast::RoadType>("invalid"), std::nullopt);

  EXPECT_EQ(strada::parser::ToString(strada::ast::RoadType::kUnknown), "unknown");
  EXPECT_EQ(strada::parser::ToString(strada::ast::RoadType::kBicycle), "bicycle");
  EXPECT_EQ(strada::parser::ToString(strada::ast::RoadType::kLowSpeed), "lowSpeed");
  EXPECT_EQ(strada::parser::ToString(strada::ast::RoadType::kPedestrian), "pedestrian");
  EXPECT_EQ(strada::parser::ToString(strada::ast::RoadType::kMotorway), "motorway");
  EXPECT_EQ(strada::parser::ToString(strada::ast::RoadType::kRural), "rural");
  EXPECT_EQ(strada::parser::ToString(strada::ast::RoadType::kTownArterial), "townArterial");
  EXPECT_EQ(strada::parser::ToString(strada::ast::RoadType::kTownCollector), "townCollector");
  EXPECT_EQ(strada::parser::ToString(strada::ast::RoadType::kTownExpressway), "townExpressway");
  EXPECT_EQ(strada::parser::ToString(strada::ast::RoadType::kTownLocal), "townLocal");
  EXPECT_EQ(strada::parser::ToString(strada::ast::RoadType::kTownPlayStreet), "townPlayStreet");
  EXPECT_EQ(strada::parser::ToString(strada::ast::RoadType::kTownPrivate), "townPrivate");
  EXPECT_EQ(strada::parser::ToString(strada::ast::RoadType::kTown), "town");
}

TEST(ParserTest, ParseRoadTypes) {
  // Arrange
  const std::string kXml = R"(<?xml version="1.0" standalone="yes"?>
<OpenDRIVE>
  <header revMajor="1" revMinor="9" name="Test Map" version="1.0" date="2026-06-14T09:00:00" north="100.0" south="-100.0" east="200.0" west="-200.0"/>
  <road id="1" length="10.0" junction="-1">
    <planView>
      <geometry s="0.0" x="0.0" y="0.0" hdg="0.0" length="10.0">
        <line/>
      </geometry>
    </planView>
    <type s="0.0" type="townLocal"/>
    <type s="5.0" type="townExpressway"/>
    <type s="2.0" type="bicycle"/>
    <type s="8.0" type="invalid_type_here"/>
    <type s="9.0"/>
  </road>
</OpenDRIVE>)";

  // Act
  auto ast_tree = strada::parser::ParseString(kXml);

  // Assert
  ASSERT_EQ(ast_tree.roads.size(), 1);
  const auto& road = ast_tree.roads[0];

  ASSERT_EQ(road.types.size(), 5);

  EXPECT_DOUBLE_EQ(road.types[0].s, 0.0);
  EXPECT_EQ(road.types[0].type, strada::ast::RoadType::kTownLocal);

  EXPECT_DOUBLE_EQ(road.types[1].s, 2.0);
  EXPECT_EQ(road.types[1].type, strada::ast::RoadType::kBicycle);

  EXPECT_DOUBLE_EQ(road.types[2].s, 5.0);
  EXPECT_EQ(road.types[2].type, strada::ast::RoadType::kTownExpressway);

  EXPECT_DOUBLE_EQ(road.types[3].s, 8.0);
  EXPECT_EQ(road.types[3].type, strada::ast::RoadType::kUnknown);

  EXPECT_DOUBLE_EQ(road.types[4].s, 9.0);
  EXPECT_EQ(road.types[4].type, strada::ast::RoadType::kUnknown);
}

TEST(ParserTest, ParseObjectsFromFixture) {
  // Arrange
  std::filesystem::path data_dir = STRADA_TEST_DATA_DIR;
  std::filesystem::path file_path = data_dir / "objects.xodr";

  // Act
  auto ast_tree = strada::parser::ParseFile(file_path);

  // Assert
  ASSERT_EQ(ast_tree.roads.size(), 1);
  const auto& road = ast_tree.roads[0];

  auto find_obj = [&](const std::string& id) -> const strada::ast::Object* {
    for (const auto& obj : road.objects) {
      if (obj.id == id) {
        return &obj;
      }
    }
    return nullptr;
  };

  // 1. Verify object "0" (building)
  {
    auto* obj = find_obj("0");
    ASSERT_NE(obj, nullptr);
    EXPECT_EQ(obj->type, strada::ast::ObjectType::kBuilding);
    EXPECT_EQ(obj->subtype, "building");
    EXPECT_EQ(obj->name, "house");
    EXPECT_DOUBLE_EQ(obj->s, 2.4028125000000038e+01);
    EXPECT_DOUBLE_EQ(obj->t, 1.2802136334240046e+01);
    EXPECT_DOUBLE_EQ(obj->z_offset, 4.9999999999998934e-03);
    EXPECT_EQ(obj->orientation, strada::ast::Orientation::kNone);
    EXPECT_DOUBLE_EQ(obj->length, 1.1300000000000001e+01);
    EXPECT_DOUBLE_EQ(obj->width, 9.9900000000000002e+00);
    EXPECT_DOUBLE_EQ(obj->height, 1.2230000000000000e+01);
    EXPECT_DOUBLE_EQ(obj->hdg, 2.6413812899682183e+00);
    EXPECT_DOUBLE_EQ(obj->pitch, 0.0);
    EXPECT_DOUBLE_EQ(obj->roll, 0.0);
  }

  // 2. Verify object "2" (surface and CRG)
  {
    auto* obj = find_obj("2");
    ASSERT_NE(obj, nullptr);
    EXPECT_EQ(obj->type, strada::ast::ObjectType::kRoadSurface);
    EXPECT_EQ(obj->subtype, "patch");
    ASSERT_TRUE(obj->surface.has_value());
    ASSERT_TRUE(obj->surface->crg.has_value());
    EXPECT_EQ(obj->surface->crg->file, "Rd_Damage_Patch_22_Center.crg");
    EXPECT_TRUE(obj->surface->crg->hide_road_surface_crg);
    EXPECT_DOUBLE_EQ(obj->surface->crg->z_scale, 1.0);
  }

  // 3. Verify object "5" (outline corners local & material & validity)
  {
    auto* obj = find_obj("5");
    ASSERT_NE(obj, nullptr);
    EXPECT_EQ(obj->type, strada::ast::ObjectType::kRoadMark);
    EXPECT_EQ(obj->subtype, "arrowStraight");

    // Outlines
    ASSERT_EQ(obj->outlines.size(), 1);
    const auto& outline = obj->outlines[0];
    EXPECT_EQ(outline.id, 1);
    EXPECT_TRUE(outline.outer);
    EXPECT_TRUE(outline.closed);
    EXPECT_EQ(outline.lane_type, "driving");

    ASSERT_EQ(outline.corners_local.size(), 8);
    EXPECT_DOUBLE_EQ(outline.corners_local[0].u, -3.6386);
    EXPECT_DOUBLE_EQ(outline.corners_local[0].v, 0.1123);
    EXPECT_DOUBLE_EQ(outline.corners_local[0].z, 0.0);
    EXPECT_DOUBLE_EQ(outline.corners_local[0].height, 0.0);
    EXPECT_EQ(outline.corners_local[0].id, 0);

    // Materials
    ASSERT_EQ(obj->materials.size(), 1);
    EXPECT_EQ(obj->materials[0].road_mark_color, "white");

    // Validity
    ASSERT_EQ(obj->validities.size(), 1);
    EXPECT_EQ(obj->validities[0].from_lane, 1);
    EXPECT_EQ(obj->validities[0].to_lane, 1);
  }

  // 4. Verify object "4000001" (skeleton vertex road)
  {
    auto* obj = find_obj("4000001");
    ASSERT_NE(obj, nullptr);
    EXPECT_EQ(obj->type, strada::ast::ObjectType::kGantry);
    ASSERT_TRUE(obj->skeleton.has_value());
    ASSERT_EQ(obj->skeleton->polylines.size(), 1);
    const auto& poly = obj->skeleton->polylines[0];
    EXPECT_EQ(poly.id, 1);
    ASSERT_EQ(poly.vertices_road.size(), 4);

    EXPECT_DOUBLE_EQ(poly.vertices_road[0].s, 230.0);
    EXPECT_DOUBLE_EQ(poly.vertices_road[0].t, -4.0);
    EXPECT_DOUBLE_EQ(poly.vertices_road[0].dz, 0.0);
    EXPECT_DOUBLE_EQ(poly.vertices_road[0].radius, 0.5);
    EXPECT_EQ(poly.vertices_road[0].id, 1);
    EXPECT_TRUE(poly.vertices_road[0].intersection_point);

    EXPECT_DOUBLE_EQ(poly.vertices_road[1].dz, 5.25);
    EXPECT_FALSE(poly.vertices_road[1].intersection_point);
  }

  // 5. Verify object "8" (corners road and border)
  {
    auto* obj = find_obj("8");
    ASSERT_NE(obj, nullptr);
    EXPECT_EQ(obj->type, strada::ast::ObjectType::kTrafficIsland);
    ASSERT_EQ(obj->outlines.size(), 1);
    const auto& outline = obj->outlines[0];
    EXPECT_EQ(outline.id, 50);
    EXPECT_EQ(outline.fill_type, "cobble");
    EXPECT_TRUE(outline.closed);
    ASSERT_EQ(outline.corners_road.size(), 30);
    EXPECT_DOUBLE_EQ(outline.corners_road[0].s, 52.5);
    EXPECT_DOUBLE_EQ(outline.corners_road[0].t, 1.5);
    EXPECT_DOUBLE_EQ(outline.corners_road[0].dz, 0.0);
    EXPECT_DOUBLE_EQ(outline.corners_road[0].height, 0.1);

    // Borders
    ASSERT_EQ(obj->borders.size(), 1);
    const auto& border = obj->borders[0];
    EXPECT_DOUBLE_EQ(border.width, 0.1);
    EXPECT_EQ(border.type, "curb");
    EXPECT_EQ(border.outline_id, 50);
    EXPECT_TRUE(border.use_complete_outline);
  }

  // 6. Verify object "9" (parking space and markings cornerReference)
  {
    auto* obj = find_obj("9");
    ASSERT_NE(obj, nullptr);
    EXPECT_EQ(obj->type, strada::ast::ObjectType::kParkingSpace);
    ASSERT_TRUE(obj->parking_space.has_value());
    EXPECT_EQ(obj->parking_space->access, "all");

    ASSERT_EQ(obj->outlines.size(), 1);
    const auto& outline = obj->outlines[0];
    EXPECT_EQ(outline.id, 51);
    ASSERT_EQ(outline.markings.size(), 2);

    const auto& marking = outline.markings[0];
    EXPECT_DOUBLE_EQ(marking.width, 0.1);
    EXPECT_EQ(marking.color, "white");
    EXPECT_DOUBLE_EQ(marking.z_offset, 0.005);
    ASSERT_EQ(marking.corner_references.size(), 2);
    EXPECT_EQ(marking.corner_references[0].id, 0);
    EXPECT_EQ(marking.corner_references[1].id, 1);
  }

  // 7. Verify object "4000203" (repeats)
  {
    auto* obj = find_obj("4000203");
    ASSERT_NE(obj, nullptr);
    EXPECT_EQ(obj->type, strada::ast::ObjectType::kBarrier);
    ASSERT_EQ(obj->repeats.size(), 12);
    const auto& rep = obj->repeats[0];
    EXPECT_DOUBLE_EQ(rep.s, 7.4615629425e+01);
    EXPECT_DOUBLE_EQ(rep.length, 2.2248725912e+00);
    EXPECT_DOUBLE_EQ(rep.distance, 0.0);
    EXPECT_DOUBLE_EQ(rep.t_start, -1.4796332056e+01);
    EXPECT_DOUBLE_EQ(rep.t_end, -1.4797490375e+01);
    EXPECT_DOUBLE_EQ(rep.width_start, 0.1);
    EXPECT_DOUBLE_EQ(rep.width_end, 0.1);
    EXPECT_DOUBLE_EQ(rep.height_start, 0.3);
    EXPECT_DOUBLE_EQ(rep.height_end, 0.3);
    EXPECT_DOUBLE_EQ(rep.z_offset_start, -3.2829728032e-01);
    EXPECT_DOUBLE_EQ(rep.z_offset_end, -2.0316925494e-01);
  }

  // 8. Verify object "6" (skeleton vertexLocal)
  {
    auto* obj = find_obj("6");
    ASSERT_NE(obj, nullptr);
    EXPECT_EQ(obj->type, strada::ast::ObjectType::kTree);
    ASSERT_TRUE(obj->skeleton.has_value());
    ASSERT_EQ(obj->skeleton->polylines.size(), 1);
    const auto& poly = obj->skeleton->polylines[0];
    ASSERT_EQ(poly.vertices_local.size(), 2);
    EXPECT_DOUBLE_EQ(poly.vertices_local[0].u, -0.2);
    EXPECT_DOUBLE_EQ(poly.vertices_local[0].v, 1.0);
    EXPECT_DOUBLE_EQ(poly.vertices_local[0].z, 1.120);
    EXPECT_DOUBLE_EQ(poly.vertices_local[0].radius, 0.15);
    EXPECT_EQ(poly.vertices_local[0].id, 1);
    EXPECT_TRUE(poly.vertices_local[0].intersection_point);
  }
}

TEST(ParserTest, ParseObjectReferenceAndValidation) {
  // Arrange
  const std::string kXml = R"(<?xml version="1.0" standalone="yes"?>
<OpenDRIVE>
  <header revMajor="1" revMinor="9" name="Test Map" version="1.0" date="2026-06-14T09:00:00"/>
  <road id="1" length="100.0" junction="-1">
    <planView>
      <geometry s="0.0" x="0.0" y="0.0" hdg="0.0" length="100.0">
        <line/>
      </geometry>
    </planView>
    <objects>
      <objectReference id="ref1" s="10.0" t="-2.0" zOffset="0.5" validLength="5.0" orientation="+" customAttr="customVal">
        <validity fromLane="1" toLane="2" layer="temporary"/>
        <userData>
          <customElement val="foo"/>
        </userData>
      </objectReference>
    </objects>
  </road>
</OpenDRIVE>)";

  // Act
  auto ast_tree = strada::parser::ParseString(kXml);

  // Assert
  ASSERT_EQ(ast_tree.roads.size(), 1);
  const auto& road = ast_tree.roads[0];
  ASSERT_EQ(road.object_references.size(), 1);

  const auto& ref = road.object_references[0];
  EXPECT_EQ(ref.id, "ref1");
  EXPECT_DOUBLE_EQ(ref.s, 10.0);
  EXPECT_DOUBLE_EQ(ref.t, -2.0);
  EXPECT_DOUBLE_EQ(ref.z_offset, 0.5);
  EXPECT_DOUBLE_EQ(ref.valid_length, 5.0);
  EXPECT_EQ(ref.orientation, strada::ast::Orientation::kPlus);

  ASSERT_EQ(ref.validities.size(), 1);
  EXPECT_EQ(ref.validities[0].from_lane, 1);
  EXPECT_EQ(ref.validities[0].to_lane, 2);
  EXPECT_EQ(ref.validities[0].layer, strada::ast::LayerType::kTemporary);

  // Extensions
  EXPECT_EQ(ref.extensions.attributes.at("customAttr"), "customVal");
  ASSERT_EQ(ref.extensions.user_data.size(), 1);
  EXPECT_NE(ref.extensions.user_data[0].find("<customElement val=\"foo\""), std::string::npos);
}

TEST(ParserTest, ObjectsRequiredAttributesThrow) {
  // Missing object id
  {
    const std::string kXml = R"(<?xml version="1.0"?>
<OpenDRIVE>
  <header revMajor="1" revMinor="9" name="Test"/>
  <road id="1" length="10.0" junction="-1">
    <planView><geometry s="0.0" x="0.0" y="0.0" hdg="0.0" length="10.0"><line/></geometry></planView>
    <objects><object s="0.0" t="0.0" orientation="none"/></objects>
  </road>
</OpenDRIVE>)";
    EXPECT_THROW(strada::parser::ParseString(kXml), strada::parser::MissingElementError);
  }

  // Missing object s
  {
    const std::string kXml = R"(<?xml version="1.0"?>
<OpenDRIVE>
  <header revMajor="1" revMinor="9" name="Test"/>
  <road id="1" length="10.0" junction="-1">
    <planView><geometry s="0.0" x="0.0" y="0.0" hdg="0.0" length="10.0"><line/></geometry></planView>
    <objects><object id="1" t="0.0" orientation="none"/></objects>
  </road>
</OpenDRIVE>)";
    EXPECT_THROW(strada::parser::ParseString(kXml), strada::parser::MissingElementError);
  }

  // Missing object t
  {
    const std::string kXml = R"(<?xml version="1.0"?>
<OpenDRIVE>
  <header revMajor="1" revMinor="9" name="Test"/>
  <road id="1" length="10.0" junction="-1">
    <planView><geometry s="0.0" x="0.0" y="0.0" hdg="0.0" length="10.0"><line/></geometry></planView>
    <objects><object id="1" s="0.0" orientation="none"/></objects>
  </road>
</OpenDRIVE>)";
    EXPECT_THROW(strada::parser::ParseString(kXml), strada::parser::MissingElementError);
  }

  // Missing object orientation
  {
    const std::string kXml = R"(<?xml version="1.0"?>
<OpenDRIVE>
  <header revMajor="1" revMinor="9" name="Test"/>
  <road id="1" length="10.0" junction="-1">
    <planView><geometry s="0.0" x="0.0" y="0.0" hdg="0.0" length="10.0"><line/></geometry></planView>
    <objects><object id="1" s="0.0" t="0.0"/></objects>
  </road>
</OpenDRIVE>)";
    EXPECT_THROW(strada::parser::ParseString(kXml), strada::parser::MissingElementError);
  }

  // Invalid object orientation
  {
    const std::string kXml = R"(<?xml version="1.0"?>
<OpenDRIVE>
  <header revMajor="1" revMinor="9" name="Test"/>
  <road id="1" length="10.0" junction="-1">
    <planView><geometry s="0.0" x="0.0" y="0.0" hdg="0.0" length="10.0"><line/></geometry></planView>
    <objects><object id="1" s="0.0" t="0.0" orientation="invalid_orientation"/></objects>
  </road>
</OpenDRIVE>)";
    EXPECT_THROW(strada::parser::ParseString(kXml), strada::parser::InvalidAttributeError);
  }

  // Invalid object type
  {
    const std::string kXml = R"(<?xml version="1.0"?>
<OpenDRIVE>
  <header revMajor="1" revMinor="9" name="Test"/>
  <road id="1" length="10.0" junction="-1">
    <planView><geometry s="0.0" x="0.0" y="0.0" hdg="0.0" length="10.0"><line/></geometry></planView>
    <objects><object id="1" s="0.0" t="0.0" orientation="none" type="invalid_type"/></objects>
  </road>
</OpenDRIVE>)";
    EXPECT_THROW(strada::parser::ParseString(kXml), strada::parser::InvalidAttributeError);
  }

  // Missing objectReference id
  {
    const std::string kXml = R"(<?xml version="1.0"?>
<OpenDRIVE>
  <header revMajor="1" revMinor="9" name="Test"/>
  <road id="1" length="10.0" junction="-1">
    <planView><geometry s="0.0" x="0.0" y="0.0" hdg="0.0" length="10.0"><line/></geometry></planView>
    <objects><objectReference s="0.0" t="0.0" orientation="none"/></objects>
  </road>
</OpenDRIVE>)";
    EXPECT_THROW(strada::parser::ParseString(kXml), strada::parser::MissingElementError);
  }

  // Missing objectReference s
  {
    const std::string kXml = R"(<?xml version="1.0"?>
<OpenDRIVE>
  <header revMajor="1" revMinor="9" name="Test"/>
  <road id="1" length="10.0" junction="-1">
    <planView><geometry s="0.0" x="0.0" y="0.0" hdg="0.0" length="10.0"><line/></geometry></planView>
    <objects><objectReference id="1" t="0.0" orientation="none"/></objects>
  </road>
</OpenDRIVE>)";
    EXPECT_THROW(strada::parser::ParseString(kXml), strada::parser::MissingElementError);
  }

  // Missing objectReference t
  {
    const std::string kXml = R"(<?xml version="1.0"?>
<OpenDRIVE>
  <header revMajor="1" revMinor="9" name="Test"/>
  <road id="1" length="10.0" junction="-1">
    <planView><geometry s="0.0" x="0.0" y="0.0" hdg="0.0" length="10.0"><line/></geometry></planView>
    <objects><objectReference id="1" s="0.0" orientation="none"/></objects>
  </road>
</OpenDRIVE>)";
    EXPECT_THROW(strada::parser::ParseString(kXml), strada::parser::MissingElementError);
  }

  // Missing objectReference orientation
  {
    const std::string kXml = R"(<?xml version="1.0"?>
<OpenDRIVE>
  <header revMajor="1" revMinor="9" name="Test"/>
  <road id="1" length="10.0" junction="-1">
    <planView><geometry s="0.0" x="0.0" y="0.0" hdg="0.0" length="10.0"><line/></geometry></planView>
    <objects><objectReference id="1" s="0.0" t="0.0"/></objects>
  </road>
</OpenDRIVE>)";
    EXPECT_THROW(strada::parser::ParseString(kXml), strada::parser::MissingElementError);
  }

  // Invalid objectReference orientation
  {
    const std::string kXml = R"(<?xml version="1.0"?>
<OpenDRIVE>
  <header revMajor="1" revMinor="9" name="Test"/>
  <road id="1" length="10.0" junction="-1">
    <planView><geometry s="0.0" x="0.0" y="0.0" hdg="0.0" length="10.0"><line/></geometry></planView>
    <objects><objectReference id="1" s="0.0" t="0.0" orientation="invalid_orientation"/></objects>
  </road>
</OpenDRIVE>)";
    EXPECT_THROW(strada::parser::ParseString(kXml), strada::parser::InvalidAttributeError);
  }
}
