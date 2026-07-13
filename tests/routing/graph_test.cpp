#include <gtest/gtest.h>

#include <strada/parser/parser.hpp>
#include <strada/routing/graph.hpp>
#include <string>
#include <vector>

namespace strada::routing::test {

TEST(RoutingGraphTest, StraightTopologyPathfinding) {
  // Arrange
  const std::string xml = R"(<?xml version="1.0" standalone="yes"?>
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
</OpenDRIVE>)";

  auto ast = strada::parser::ParseString(xml);
  Graph graph(ast);

  // Act
  auto path = graph.FindPath("1", "2");

  // Assert
  ASSERT_TRUE(path.has_value());
  EXPECT_EQ(path->size(), 2);
  EXPECT_EQ((*path)[0], "1");
  EXPECT_EQ((*path)[1], "2");
}

TEST(RoutingGraphTest, OneWayRoadPreventsWrongWay) {
  // Arrange
  // Road 1 (driving lane right id=-1 -> forward only)
  // Road 2 (driving lane left id=1 -> backward only)
  const std::string xml = R"(<?xml version="1.0" standalone="yes"?>
<OpenDRIVE>
  <header revMajor="1" revMinor="9"/>
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
        <left>
          <lane id="1" type="driving"><width sOffset="0.0" a="3.0" b="0.0" c="0.0" d="0.0"/></lane>
        </left>
      </laneSection>
    </lanes>
  </road>
</OpenDRIVE>)";

  auto ast = strada::parser::ParseString(xml);
  Graph graph(ast);

  // Act & Assert
  // Cannot travel 1 -> 2 because 2 is backward-only (positive ID)
  auto path12 = graph.FindPath("1", "2");
  EXPECT_FALSE(path12.has_value());

  // Can travel 2 -> 1 because we can go backward on 2 (leaving at start) and forward on 1 (entering at start)
  // Wait, let's trace:
  // 2 (backward, leaves at start) connects to 1 (predecessor of 2 is 1 with contact point end, which means start of 2
  // connects to end of 1. Entering 1 at end means we must travel backward on 1. But 1 only has a forward lane (negative
  // ID). So 2 -> 1 is also blocked because 1 does not have a backward lane!
  auto path21 = graph.FindPath("2", "1");
  EXPECT_FALSE(path21.has_value());
}

TEST(RoutingGraphTest, LoopTopologyShortestPath) {
  // Arrange
  // 1 -> 2 -> 3
  // 1 -> 4 -> 3 (longer)
  const std::string xml = R"(<?xml version="1.0" standalone="yes"?>
<OpenDRIVE>
  <header revMajor="1" revMinor="9"/>
  <road id="1" length="10.0" junction="-1">
    <link>
      <successor elementType="road" elementId="2" contactPoint="start"/>
    </link>
    <planView>
      <geometry s="0.0" x="0.0" y="0.0" hdg="0.0" length="10.0"><line/></geometry>
    </planView>
    <lanes>
      <laneSection s="0.0">
        <right><lane id="-1" type="driving"><width sOffset="0.0" a="3.0"/></lane></right>
      </laneSection>
    </lanes>
  </road>
  <road id="2" length="5.0" junction="-1">
    <link>
      <predecessor elementType="road" elementId="1" contactPoint="end"/>
      <successor elementType="road" elementId="3" contactPoint="start"/>
    </link>
    <planView>
      <geometry s="0.0" x="10.0" y="0.0" hdg="0.0" length="5.0"><line/></geometry>
    </planView>
    <lanes>
      <laneSection s="0.0">
        <right><lane id="-1" type="driving"><width sOffset="0.0" a="3.0"/></lane></right>
      </laneSection>
    </lanes>
  </road>
  <road id="4" length="50.0" junction="-1">
    <link>
      <predecessor elementType="road" elementId="1" contactPoint="end"/>
      <successor elementType="road" elementId="3" contactPoint="start"/>
    </link>
    <planView>
      <geometry s="0.0" x="10.0" y="0.0" hdg="0.0" length="50.0"><line/></geometry>
    </planView>
    <lanes>
      <laneSection s="0.0">
        <right><lane id="-1" type="driving"><width sOffset="0.0" a="3.0"/></lane></right>
      </laneSection>
    </lanes>
  </road>
  <road id="3" length="10.0" junction="-1">
    <link>
      <predecessor elementType="road" elementId="2" contactPoint="end"/>
    </link>
    <planView>
      <geometry s="0.0" x="15.0" y="0.0" hdg="0.0" length="10.0"><line/></geometry>
    </planView>
    <lanes>
      <laneSection s="0.0">
        <right><lane id="-1" type="driving"><width sOffset="0.0" a="3.0"/></lane></right>
      </laneSection>
    </lanes>
  </road>
</OpenDRIVE>)";

  auto ast = strada::parser::ParseString(xml);
  Graph graph(ast);

  // Act
  auto path = graph.FindPath("1", "3");

  // Assert
  ASSERT_TRUE(path.has_value());
  EXPECT_EQ(path->size(), 3);
  EXPECT_EQ((*path)[0], "1");
  EXPECT_EQ((*path)[1], "2");
  EXPECT_EQ((*path)[2], "3");
}

TEST(RoutingGraphTest, JunctionConnectingRoad) {
  // Arrange
  // Road 1 -> Junction 10 (Connecting Road 3) -> Road 2
  const std::string xml = R"(<?xml version="1.0" standalone="yes"?>
<OpenDRIVE>
  <header revMajor="1" revMinor="9"/>
  <road id="1" length="10.0" junction="-1">
    <link>
      <successor elementType="junction" elementId="10"/>
    </link>
    <planView>
      <geometry s="0.0" x="0.0" y="0.0" hdg="0.0" length="10.0"><line/></geometry>
    </planView>
    <lanes>
      <laneSection s="0.0">
        <right><lane id="-1" type="driving"><width sOffset="0.0" a="3.0"/></lane></right>
      </laneSection>
    </lanes>
  </road>
  <road id="3" length="5.0" junction="10">
    <link>
      <successor elementType="road" elementId="2" contactPoint="start"/>
    </link>
    <planView>
      <geometry s="0.0" x="10.0" y="0.0" hdg="0.0" length="5.0"><line/></geometry>
    </planView>
    <lanes>
      <laneSection s="0.0">
        <right><lane id="-1" type="driving"><width sOffset="0.0" a="3.0"/></lane></right>
      </laneSection>
    </lanes>
  </road>
  <road id="2" length="10.0" junction="-1">
    <link>
      <predecessor elementType="road" elementId="3" contactPoint="end"/>
    </link>
    <planView>
      <geometry s="0.0" x="15.0" y="0.0" hdg="0.0" length="10.0"><line/></geometry>
    </planView>
    <lanes>
      <laneSection s="0.0">
        <right><lane id="-1" type="driving"><width sOffset="0.0" a="3.0"/></lane></right>
      </laneSection>
    </lanes>
  </road>
  <junction id="10">
    <connection id="1" incomingRoad="1" connectingRoad="3" contactPoint="start"/>
  </junction>
</OpenDRIVE>)";

  auto ast = strada::parser::ParseString(xml);
  Graph graph(ast);

  // Act
  auto path = graph.FindPath("1", "2");

  // Assert
  ASSERT_TRUE(path.has_value());
  EXPECT_EQ(path->size(), 3);
  EXPECT_EQ((*path)[0], "1");
  EXPECT_EQ((*path)[1], "3");
  EXPECT_EQ((*path)[2], "2");
}

TEST(RoutingGraphTest, CostFunctorJunctionPenalty) {
  // Arrange
  // 1 -> 2 -> 3 (via junction road 2, total length 10 + 5 + 10 = 25)
  // 1 -> 4 -> 3 (direct road 4, total length 10 + 12 + 10 = 32)
  const std::string xml = R"(<?xml version="1.0" standalone="yes"?>
<OpenDRIVE>
  <header revMajor="1" revMinor="9"/>
  <road id="1" length="10.0" junction="-1">
    <link>
      <successor elementType="road" elementId="4" contactPoint="start"/>
    </link>
    <planView>
      <geometry s="0.0" x="0.0" y="0.0" hdg="0.0" length="10.0"><line/></geometry>
    </planView>
    <lanes>
      <laneSection s="0.0">
        <right><lane id="-1" type="driving"><width sOffset="0.0" a="3.0"/></lane></right>
      </laneSection>
    </lanes>
  </road>
  <road id="2" length="5.0" junction="10">
    <link>
      <successor elementType="road" elementId="3" contactPoint="start"/>
    </link>
    <planView>
      <geometry s="0.0" x="10.0" y="0.0" hdg="0.0" length="5.0"><line/></geometry>
    </planView>
    <lanes>
      <laneSection s="0.0">
        <right><lane id="-1" type="driving"><width sOffset="0.0" a="3.0"/></lane></right>
      </laneSection>
    </lanes>
  </road>
  <road id="4" length="12.0" junction="-1">
    <link>
      <predecessor elementType="road" elementId="1" contactPoint="end"/>
      <successor elementType="road" elementId="3" contactPoint="start"/>
    </link>
    <planView>
      <geometry s="0.0" x="10.0" y="0.0" hdg="0.0" length="12.0"><line/></geometry>
    </planView>
    <lanes>
      <laneSection s="0.0">
        <right><lane id="-1" type="driving"><width sOffset="0.0" a="3.0"/></lane></right>
      </laneSection>
    </lanes>
  </road>
  <road id="3" length="10.0" junction="-1">
    <link>
      <predecessor elementType="road" elementId="4" contactPoint="end"/>
    </link>
    <planView>
      <geometry s="0.0" x="15.0" y="0.0" hdg="0.0" length="10.0"><line/></geometry>
    </planView>
    <lanes>
      <laneSection s="0.0">
        <right><lane id="-1" type="driving"><width sOffset="0.0" a="3.0"/></lane></right>
      </laneSection>
    </lanes>
  </road>
  <junction id="10">
    <connection id="1" incomingRoad="1" connectingRoad="2" contactPoint="start"/>
  </junction>
</OpenDRIVE>)";

  auto ast = strada::parser::ParseString(xml);
  Graph graph(ast);

  // Without penalty, path should go through junction (1 -> 2 -> 3) since 10+5+10 = 25 < 32
  auto path_no_penalty = graph.FindPath("1", "3");
  ASSERT_TRUE(path_no_penalty.has_value());
  EXPECT_EQ((*path_no_penalty)[1], "2");

  // With a junction penalty of 10.0:
  // Cost of 1 -> 2 -> 3 is: 10 + (5 + 10) + 10 = 35
  // Cost of 1 -> 4 -> 3 is: 10 + 12 + 10 = 32
  // So the pathfinder should prefer 1 -> 4 -> 3!
  auto path_with_penalty = graph.FindPath("1", "3", [&](std::string_view road_id) {
    double cost = graph.GetRoadLength(road_id);
    if (graph.IsJunctionRoad(road_id)) {
      cost += 10.0;
    }
    return cost;
  });

  ASSERT_TRUE(path_with_penalty.has_value());
  EXPECT_EQ(path_with_penalty->size(), 3);
  EXPECT_EQ((*path_with_penalty)[0], "1");
  EXPECT_EQ((*path_with_penalty)[1], "4");
  EXPECT_EQ((*path_with_penalty)[2], "3");
}

}  // namespace strada::routing::test
