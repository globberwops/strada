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
  const auto& header = opendrive.header_;

  // Assert
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
  std::filesystem::remove(temp_filename);
  const auto& header = opendrive.header_;

  // Assert
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
  const auto& roads = opendrive.roads_;

  // Assert
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
