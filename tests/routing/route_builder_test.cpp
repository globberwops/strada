// This file contains unit tests for the RouteBuilder class.

#include <gtest/gtest.h>

#include <strada/parser/parser.hpp>
#include <strada/routing/graph.hpp>
#include <strada/routing/route_builder.hpp>
#include <string>
#include <vector>

namespace strada::routing::test {

TEST(RouteBuilderTest, EmptyState) {
  const auto xml = std::string(R"(<?xml version="1.0" standalone="yes"?>
<OpenDRIVE>
  <header revMajor="1" revMinor="9" name="Test Map" version="1.0" date="2026-06-14T09:00:00"/>
  <road id="1" length="10.0" junction="-1">
    <link/>
    <planView>
      <geometry s="0.0" x="0.0" y="0.0" hdg="0.0" length="10.0"><line/></geometry>
    </planView>
    <lanes>
      <laneSection s="0.0">
        <right>
          <lane id="-1" type="driving"><width sOffset="0.0" a="3.0" b="0.0" c="0.0" d="0.0"/></lane>
        </right>
      </laneSection>
    </lanes>
  </road>
</OpenDRIVE>)");

  const auto ast = strada::parser::ParseString(xml);
  const auto graph = Graph{ast};
  auto builder = RouteBuilder{graph};

  EXPECT_TRUE(builder.Waypoints().empty());
  EXPECT_FALSE(builder.ActiveRoute().has_value());
  EXPECT_TRUE(builder.RouteError().empty());
}

TEST(RouteBuilderTest, AppendSingleWaypoint) {
  const auto xml = std::string(R"(<?xml version="1.0" standalone="yes"?>
<OpenDRIVE>
  <header revMajor="1" revMinor="9" name="Test Map" version="1.0" date="2026-06-14T09:00:00"/>
  <road id="1" length="10.0" junction="-1">
    <link/>
    <planView>
      <geometry s="0.0" x="0.0" y="0.0" hdg="0.0" length="10.0"><line/></geometry>
    </planView>
    <lanes>
      <laneSection s="0.0">
        <right>
          <lane id="-1" type="driving"><width sOffset="0.0" a="3.0" b="0.0" c="0.0" d="0.0"/></lane>
        </right>
      </laneSection>
    </lanes>
  </road>
</OpenDRIVE>)");

  const auto ast = strada::parser::ParseString(xml);
  const auto graph = Graph{ast};
  auto builder = RouteBuilder{graph};

  EXPECT_TRUE(builder.AppendWaypoint("1"));
  ASSERT_EQ(builder.Waypoints().size(), 1);
  EXPECT_EQ(builder.Waypoints()[0], "1");
  EXPECT_FALSE(builder.ActiveRoute().has_value());
  EXPECT_TRUE(builder.RouteError().empty());
}

TEST(RouteBuilderTest, AppendValidRoute) {
  const auto xml = std::string(R"(<?xml version="1.0" standalone="yes"?>
<OpenDRIVE>
  <header revMajor="1" revMinor="9" name="Test Map" version="1.0" date="2026-06-14T09:00:00"/>
  <road id="1" length="10.0" junction="-1">
    <link>
      <successor elementType="road" elementId="2" contactPoint="start"/>
    </link>
    <planView>
      <geometry s="0.0" x="0.0" y="0.0" hdg="0.0" length="10.0"><line/></geometry>
    </planView>
    <lanes>
      <laneSection s="0.0">
        <right>
          <lane id="-1" type="driving"><width sOffset="0.0" a="3.0" b="0.0" c="0.0" d="0.0"/></lane>
        </right>
      </laneSection>
    </lanes>
  </road>
  <road id="2" length="20.0" junction="-1">
    <link>
      <predecessor elementType="road" elementId="1" contactPoint="end"/>
    </link>
    <planView>
      <geometry s="0.0" x="10.0" y="0.0" hdg="0.0" length="20.0"><line/></geometry>
    </planView>
    <lanes>
      <laneSection s="0.0">
        <right>
          <lane id="-1" type="driving"><width sOffset="0.0" a="3.0" b="0.0" c="0.0" d="0.0"/></lane>
        </right>
      </laneSection>
    </lanes>
  </road>
</OpenDRIVE>)");

  const auto ast = strada::parser::ParseString(xml);
  const auto graph = Graph{ast};
  auto builder = RouteBuilder{graph};

  EXPECT_TRUE(builder.AppendWaypoint("1"));
  EXPECT_TRUE(builder.AppendWaypoint("2"));

  ASSERT_EQ(builder.Waypoints().size(), 2);
  EXPECT_EQ(builder.Waypoints()[0], "1");
  EXPECT_EQ(builder.Waypoints()[1], "2");

  ASSERT_TRUE(builder.ActiveRoute().has_value());
  EXPECT_EQ(builder.ActiveRoute()->segments.size(), 2);
  EXPECT_EQ(builder.ActiveRoute()->segments[0].road_id, "1");
  EXPECT_EQ(builder.ActiveRoute()->segments[1].road_id, "2");
  EXPECT_DOUBLE_EQ(builder.ActiveRoute()->segments[0].length, 10.0);
  EXPECT_DOUBLE_EQ(builder.ActiveRoute()->segments[1].length, 20.0);
  EXPECT_TRUE(builder.RouteError().empty());
}

TEST(RouteBuilderTest, AppendInvalidRoute) {
  const auto xml = std::string(R"(<?xml version="1.0" standalone="yes"?>
<OpenDRIVE>
  <header revMajor="1" revMinor="9" name="Test Map" version="1.0" date="2026-06-14T09:00:00"/>
  <road id="1" length="10.0" junction="-1">
    <planView>
      <geometry s="0.0" x="0.0" y="0.0" hdg="0.0" length="10.0"><line/></geometry>
    </planView>
    <lanes>
      <laneSection s="0.0">
        <right>
          <lane id="-1" type="driving"><width sOffset="0.0" a="3.0" b="0.0" c="0.0" d="0.0"/></lane>
        </right>
      </laneSection>
    </lanes>
  </road>
  <road id="2" length="20.0" junction="-1">
    <planView>
      <geometry s="0.0" x="10.0" y="20.0" hdg="0.0" length="20.0"><line/></geometry>
    </planView>
    <lanes>
      <laneSection s="0.0">
        <right>
          <lane id="-1" type="driving"><width sOffset="0.0" a="3.0" b="0.0" c="0.0" d="0.0"/></lane>
        </right>
      </laneSection>
    </lanes>
  </road>
</OpenDRIVE>)");

  const auto ast = strada::parser::ParseString(xml);
  const auto graph = Graph{ast};
  auto builder = RouteBuilder{graph};

  EXPECT_TRUE(builder.AppendWaypoint("1"));
  EXPECT_FALSE(builder.AppendWaypoint("2"));

  ASSERT_EQ(builder.Waypoints().size(), 2);
  EXPECT_FALSE(builder.ActiveRoute().has_value());
  EXPECT_EQ(builder.RouteError(), "No path found between road 1 and 2");
}

TEST(RouteBuilderTest, UndoAndClear) {
  const auto xml = std::string(R"(<?xml version="1.0" standalone="yes"?>
<OpenDRIVE>
  <header revMajor="1" revMinor="9" name="Test Map" version="1.0" date="2026-06-14T09:00:00"/>
  <road id="1" length="10.0" junction="-1">
    <link>
      <successor elementType="road" elementId="2" contactPoint="start"/>
    </link>
    <planView>
      <geometry s="0.0" x="0.0" y="0.0" hdg="0.0" length="10.0"><line/></geometry>
    </planView>
    <lanes>
      <laneSection s="0.0">
        <right>
          <lane id="-1" type="driving"><width sOffset="0.0" a="3.0" b="0.0" c="0.0" d="0.0"/></lane>
        </right>
      </laneSection>
    </lanes>
  </road>
  <road id="2" length="20.0" junction="-1">
    <link>
      <predecessor elementType="road" elementId="1" contactPoint="end"/>
    </link>
    <planView>
      <geometry s="0.0" x="10.0" y="0.0" hdg="0.0" length="20.0"><line/></geometry>
    </planView>
    <lanes>
      <laneSection s="0.0">
        <right>
          <lane id="-1" type="driving"><width sOffset="0.0" a="3.0" b="0.0" c="0.0" d="0.0"/></lane>
        </right>
      </laneSection>
    </lanes>
  </road>
</OpenDRIVE>)");

  const auto ast = strada::parser::ParseString(xml);
  const auto graph = Graph{ast};
  auto builder = RouteBuilder{graph};

  // Add 1, 2
  builder.AppendWaypoint("1");
  builder.AppendWaypoint("2");
  EXPECT_TRUE(builder.ActiveRoute().has_value());

  // Undo back to 1
  builder.Undo();
  EXPECT_EQ(builder.Waypoints().size(), 1);
  EXPECT_FALSE(builder.ActiveRoute().has_value());
  EXPECT_TRUE(builder.RouteError().empty());

  // Undo back to 0
  builder.Undo();
  EXPECT_TRUE(builder.Waypoints().empty());

  // Undo on empty (noop)
  builder.Undo();
  EXPECT_TRUE(builder.Waypoints().empty());

  // Clear
  builder.AppendWaypoint("1");
  builder.AppendWaypoint("2");
  EXPECT_TRUE(builder.ActiveRoute().has_value());
  builder.Clear();
  EXPECT_TRUE(builder.Waypoints().empty());
  EXPECT_FALSE(builder.ActiveRoute().has_value());
  EXPECT_TRUE(builder.RouteError().empty());
}

}  // namespace strada::routing::test
