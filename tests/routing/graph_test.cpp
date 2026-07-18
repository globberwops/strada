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

TEST(RoutingGraphTest, FindRouteStraightTopology) {
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
  auto route = graph.FindRoute("1", "2");

  // Assert
  ASSERT_TRUE(route.has_value());
  EXPECT_EQ(route->segments.size(), 2);
  EXPECT_EQ(route->segments[0].road_id, "1");
  EXPECT_TRUE(route->segments[0].forward);
  EXPECT_DOUBLE_EQ(route->segments[0].length, 10.0);

  EXPECT_EQ(route->segments[1].road_id, "2");
  EXPECT_TRUE(route->segments[1].forward);
  EXPECT_DOUBLE_EQ(route->segments[1].length, 20.0);
}

TEST(RoutingGraphTest, FindRouteDirectionResolution) {
  // Arrange
  // Road 2 (backward only, left lane) predecessor connects to Road 1 (forward only, right lane) start.
  // This means travelling from Road 2 to Road 1 requires travelling backward on Road 2,
  // then transitioning to Road 1 and travelling forward on Road 1.
  const std::string xml = R"(<?xml version="1.0" standalone="yes"?>
<OpenDRIVE>
  <header revMajor="1" revMinor="9"/>
  <road id="1" length="10.0" junction="-1">
    <planView>
      <geometry s="0.0" x="0.0" y="0.0" hdg="0.0" length="10.0"><line/></geometry>
    </planView>
    <lanes>
      <laneSection s="0.0">
        <right>
          <lane id="-1" type="driving"><width sOffset="0.0" a="3.0"/></lane>
        </right>
      </laneSection>
    </lanes>
  </road>
  <road id="2" length="20.0" junction="-1">
    <link>
      <predecessor elementType="road" elementId="1" contactPoint="start"/>
    </link>
    <planView>
      <geometry s="0.0" x="10.0" y="0.0" hdg="3.14159" length="20.0"><line/></geometry>
    </planView>
    <lanes>
      <laneSection s="0.0">
        <left>
          <lane id="1" type="driving"><width sOffset="0.0" a="3.0"/></lane>
        </left>
      </laneSection>
    </lanes>
  </road>
</OpenDRIVE>)";

  auto ast = strada::parser::ParseString(xml);
  Graph graph(ast);

  // Act
  auto route = graph.FindRoute("2", "1");

  // Assert
  ASSERT_TRUE(route.has_value());
  EXPECT_EQ(route->segments.size(), 2);
  EXPECT_EQ(route->segments[0].road_id, "2");
  EXPECT_FALSE(route->segments[0].forward);
  EXPECT_DOUBLE_EQ(route->segments[0].length, 20.0);

  EXPECT_EQ(route->segments[1].road_id, "1");
  EXPECT_TRUE(route->segments[1].forward);
  EXPECT_DOUBLE_EQ(route->segments[1].length, 10.0);
}

TEST(RoutingGraphTest, FindRouteLoopTopology) {
  // Arrange
  // 1 -> 2 -> 3 (total length 25)
  // 1 -> 4 -> 3 (total length 70)
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
  auto route = graph.FindRoute("1", "3");

  // Assert
  ASSERT_TRUE(route.has_value());
  EXPECT_EQ(route->segments.size(), 3);
  EXPECT_EQ(route->segments[0].road_id, "1");
  EXPECT_TRUE(route->segments[0].forward);
  EXPECT_DOUBLE_EQ(route->segments[0].length, 10.0);

  EXPECT_EQ(route->segments[1].road_id, "2");
  EXPECT_TRUE(route->segments[1].forward);
  EXPECT_DOUBLE_EQ(route->segments[1].length, 5.0);

  EXPECT_EQ(route->segments[2].road_id, "3");
  EXPECT_TRUE(route->segments[2].forward);
  EXPECT_DOUBLE_EQ(route->segments[2].length, 10.0);
}

TEST(RoutingGraphTest, FindRouteDisconnectedNetwork) {
  // Arrange
  // Road 1 and Road 2 are completely disconnected.
  const std::string xml = R"(<?xml version="1.0" standalone="yes"?>
<OpenDRIVE>
  <header revMajor="1" revMinor="9"/>
  <road id="1" length="10.0" junction="-1">
    <planView>
      <geometry s="0.0" x="0.0" y="0.0" hdg="0.0" length="10.0"><line/></geometry>
    </planView>
    <lanes>
      <laneSection s="0.0">
        <right>
          <lane id="-1" type="driving"><width sOffset="0.0" a="3.0"/></lane>
        </right>
      </laneSection>
    </lanes>
  </road>
  <road id="2" length="20.0" junction="-1">
    <planView>
      <geometry s="0.0" x="50.0" y="0.0" hdg="0.0" length="20.0"><line/></geometry>
    </planView>
    <lanes>
      <laneSection s="0.0">
        <right>
          <lane id="-1" type="driving"><width sOffset="0.0" a="3.0"/></lane>
        </right>
      </laneSection>
    </lanes>
  </road>
</OpenDRIVE>)";

  auto ast = strada::parser::ParseString(xml);
  Graph graph(ast);

  // Act
  auto route = graph.FindRoute("1", "2");

  // Assert
  EXPECT_FALSE(route.has_value());
}

TEST(RoutingGraphTest, FindRouteSameStartAndEndRoad) {
  // Arrange
  // Road 1: forward only drivable (right lane)
  // Road 2: backward only drivable (left lane)
  // Road 3: not drivable (no lanes)
  const std::string xml = R"(<?xml version="1.0" standalone="yes"?>
<OpenDRIVE>
  <header revMajor="1" revMinor="9"/>
  <road id="1" length="10.0" junction="-1">
    <planView>
      <geometry s="0.0" x="0.0" y="0.0" hdg="0.0" length="10.0"><line/></geometry>
    </planView>
    <lanes>
      <laneSection s="0.0">
        <right>
          <lane id="-1" type="driving"><width sOffset="0.0" a="3.0"/></lane>
        </right>
      </laneSection>
    </lanes>
  </road>
  <road id="2" length="20.0" junction="-1">
    <planView>
      <geometry s="0.0" x="50.0" y="0.0" hdg="0.0" length="20.0"><line/></geometry>
    </planView>
    <lanes>
      <laneSection s="0.0">
        <left>
          <lane id="1" type="driving"><width sOffset="0.0" a="3.0"/></lane>
        </left>
      </laneSection>
    </lanes>
  </road>
  <road id="3" length="30.0" junction="-1">
    <planView>
      <geometry s="0.0" x="100.0" y="0.0" hdg="0.0" length="30.0"><line/></geometry>
    </planView>
    <lanes>
      <laneSection s="0.0">
      </laneSection>
    </lanes>
  </road>
</OpenDRIVE>)";

  auto ast = strada::parser::ParseString(xml);
  Graph graph(ast);

  // Act & Assert 1: Road 1 (forward only)
  auto route1 = graph.FindRoute("1", "1");
  ASSERT_TRUE(route1.has_value());
  EXPECT_EQ(route1->segments.size(), 1);
  EXPECT_EQ(route1->segments[0].road_id, "1");
  EXPECT_TRUE(route1->segments[0].forward);
  EXPECT_DOUBLE_EQ(route1->segments[0].length, 10.0);

  // Act & Assert 2: Road 2 (backward only)
  auto route2 = graph.FindRoute("2", "2");
  ASSERT_TRUE(route2.has_value());
  EXPECT_EQ(route2->segments.size(), 1);
  EXPECT_EQ(route2->segments[0].road_id, "2");
  EXPECT_FALSE(route2->segments[0].forward);
  EXPECT_DOUBLE_EQ(route2->segments[0].length, 20.0);

  // Act & Assert 3: Road 3 (not drivable)
  auto route3 = graph.FindRoute("3", "3");
  EXPECT_FALSE(route3.has_value());
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
  auto path_with_penalty = graph.FindPath("1", "3", [&](std::string_view road_id) -> double {
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

TEST(RoutingGraphTest, MotorwayExitEntryConnection9to16) {
  const auto path =
      std::filesystem::path("/workspaces/strada/.scratch/use_cases/UC_Motorway-Exit-Entry/UC_Motorway-Exit-Entry.xodr");
  auto ast = strada::parser::ParseFile(path);
  Graph graph(ast);

  auto route = graph.FindRoute("9", "16");
  ASSERT_TRUE(route.has_value());
  for (const auto& seg : route->segments) {
    std::cout << "[ROUTE_SEGMENT] " << seg.road_id << " (" << (seg.forward ? "forward" : "backward") << ")\n";
  }
}

TEST(RoutingGraphTest, MotorwayExitEntryDirectJunctionConnection9to16) {
  const auto path = std::filesystem::path(
      "/workspaces/strada/.scratch/use_cases/UC_Motorway-Exit-Entry/UC_Motorway-Exit-Entry-DirectJunction.xodr");
  auto ast = strada::parser::ParseFile(path);
  Graph graph(ast);

  auto route = graph.FindRoute("9", "16");
  ASSERT_TRUE(route.has_value());
  // Expecting path: 9 (backward) -> 32 (forward) -> 16 (forward)
  ASSERT_EQ(route->segments.size(), 3);
  EXPECT_EQ(route->segments[0].road_id, "9");
  EXPECT_FALSE(route->segments[0].forward);
  EXPECT_EQ(route->segments[1].road_id, "32");
  EXPECT_TRUE(route->segments[1].forward);
  EXPECT_EQ(route->segments[2].road_id, "16");
  EXPECT_TRUE(route->segments[2].forward);
}

}  // namespace strada::routing::test
