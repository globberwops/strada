#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
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
  std::filesystem::path data_dir = STRADA_TEST_DATA_DIR;
  std::filesystem::path file_path = data_dir / "minimal_header.xodr";

  // Act
  auto opendrive = strada::parser::ParseFile(file_path);

  // Assert
  const auto& header = opendrive.header;
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
  auto opendrive = strada::parser::ParseString(xml);

  // Assert
  const auto& roads = opendrive.roads;
  ASSERT_EQ(roads.size(), 2);

  EXPECT_EQ(roads[0].id, "1");
  EXPECT_DOUBLE_EQ(roads[0].length, 10.0);
  EXPECT_EQ(roads[0].junction, "-1");
  EXPECT_EQ(roads[0].rule, strada::ast::TrafficRule::kRht);
  EXPECT_EQ(roads[0].name, "Road 1");

  EXPECT_EQ(roads[1].id, "2");
  EXPECT_DOUBLE_EQ(roads[1].length, 25.5);
  EXPECT_EQ(roads[1].junction, "42");
  EXPECT_EQ(roads[1].rule, strada::ast::TrafficRule::kLht);
  EXPECT_EQ(roads[1].name, "");
}

TEST(ParserTest, ParseGeometryFromPlanView) {
  // Arrange
  std::filesystem::path data_dir = STRADA_TEST_DATA_DIR;
  std::string xml = ReadFileToString(data_dir / "geometry.xodr");

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
  EXPECT_EQ(param_poly3.p_range, strada::ast::PRange::kArcLength);
}

TEST(ParserTest, ParseLanesAndProfiles) {
  // Arrange
  std::filesystem::path data_dir = STRADA_TEST_DATA_DIR;
  std::string xml = ReadFileToString(data_dir / "lanes_and_profiles.xodr");

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

TEST(ParserTest, ParseJunctions) {
  // Arrange
  std::filesystem::path data_dir = STRADA_TEST_DATA_DIR;
  std::string xml = ReadFileToString(data_dir / "junctions.xodr");

  // Act
  auto opendrive = strada::parser::ParseString(xml);

  // Assert
  ASSERT_EQ(opendrive.junctions.size(), 1);
  const auto& junction = opendrive.junctions[0];
  EXPECT_EQ(junction.id, "1");
  EXPECT_EQ(junction.name, "Main Junction");
  EXPECT_EQ(junction.type, "default");

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

TEST(ParserTest, ParseExtensions) {
  // Arrange
  std::filesystem::path data_dir = STRADA_TEST_DATA_DIR;
  std::string xml = ReadFileToString(data_dir / "extensions.xodr");

  // Act
  auto opendrive = strada::parser::ParseString(xml);

  // Assert – header captures unknown attributes and userData
  const auto& header_ext = opendrive.header.extensions;
  ASSERT_EQ(header_ext.attributes.size(), 1);
  EXPECT_EQ(header_ext.attributes.at("customKey"), "customValue");
  ASSERT_EQ(header_ext.user_data.size(), 1);
  EXPECT_NE(header_ext.user_data[0].find("someVendorData"), std::string::npos);

  // Assert – road captures unknown attributes and userData
  ASSERT_EQ(opendrive.roads.size(), 1);
  const auto& road_ext = opendrive.roads[0].extensions;
  ASSERT_EQ(road_ext.attributes.size(), 1);
  EXPECT_EQ(road_ext.attributes.at("customAttr"), "roadExtra");
  ASSERT_EQ(road_ext.user_data.size(), 1);
  EXPECT_NE(road_ext.user_data[0].find("roadVendorTag"), std::string::npos);
}
