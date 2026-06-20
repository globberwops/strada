// Copyright 2026 Google LLC
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <filesystem>
#include <strada/ast/abstract_syntax_tree.hpp>
#include <strada/cpm/ids.hpp>
#include <strada/cpm/lane_network.hpp>
#include <strada/cpm/query_context.hpp>
#include <strada/parser/parser.hpp>

namespace strada::cpm {

TEST(LaneNetworkTest, LanesCompilationAndInspection) {
  // Arrange
  std::filesystem::path data_dir = STRADA_TEST_DATA_DIR;
  std::filesystem::path file_path = data_dir / "lanes_and_profiles.xodr";
  auto ast = strada::parser::ParseFile(file_path);

  // Act
  auto lane_network = strada::cpm::LaneNetwork::Build(ast);

  // Assert
  EXPECT_EQ(lane_network.LaneCount(), 3);

  auto lane0 = strada::cpm::LaneId{0};
  auto lane1 = strada::cpm::LaneId{1};
  auto lane2 = strada::cpm::LaneId{2};

  EXPECT_EQ(lane_network.OriginalLaneId(lane0), -1);
  EXPECT_EQ(lane_network.OriginalLaneId(lane1), 0);
  EXPECT_EQ(lane_network.OriginalLaneId(lane2), 1);

  EXPECT_EQ(lane_network.LaneRoad(lane0), strada::cpm::RoadId{0});
  EXPECT_EQ(lane_network.LaneRoad(lane1), strada::cpm::RoadId{0});
  EXPECT_EQ(lane_network.LaneRoad(lane2), strada::cpm::RoadId{0});

  // Verify Lane widths
  // Lane 1 (original 1): width at s = 10.0 is 3.0 + 0.1 * 10.0 = 4.0
  EXPECT_NEAR(lane_network.LaneWidth(lane2, 10.0), 4.0, 1e-9);

  // Lane -1 (original -1): width at s = 10.0 (relative ds = 9.0) is 3.2 + 0.2 * 9.0 = 5.0
  EXPECT_NEAR(lane_network.LaneWidth(lane0, 10.0), 5.0, 1e-9);

  // Center Lane (original 0): width is 0.0
  EXPECT_NEAR(lane_network.LaneWidth(lane1, 10.0), 0.0, 1e-9);
}

TEST(LaneNetworkTest, LaneTransforms) {
  // Arrange
  std::filesystem::path data_dir = STRADA_TEST_DATA_DIR;
  std::filesystem::path file_path = data_dir / "lanes_and_profiles.xodr";
  auto ast = strada::parser::ParseFile(file_path);
  auto lane_network = strada::cpm::LaneNetwork::Build(ast);

  auto lane0 = strada::cpm::LaneId{0};  // Original ID: -1
  auto lane2 = strada::cpm::LaneId{2};  // Original ID: 1

  strada::cpm::QueryContext ctx;

  // Act & Assert 1: Query left lane (lane2, original ID 1) at s = 10.0, t = 0.0, h = 0.0
  {
    strada::cpm::LanePose pose;
    pose.s = 10.0;
    pose.t = 0.0;
    pose.h = 0.0;
    pose.heading = 0.0;
    pose.pitch = 0.0;
    pose.roll = 0.0;
    pose.road = strada::cpm::RoadId{0};
    pose.lane = lane2;

    auto road_pose = lane_network.LaneToRoad(pose, ctx);
    EXPECT_NEAR(road_pose.s, 10.0, 1e-9);
    EXPECT_NEAR(road_pose.t, 878.5, 1e-9);
    EXPECT_NEAR(road_pose.h, 0.05, 1e-9);
    EXPECT_EQ(road_pose.road, strada::cpm::RoadId{0});

    auto lane_pose_opt = lane_network.RoadToLane(road_pose, ctx);
    ASSERT_TRUE(lane_pose_opt.has_value());
    auto lane_pose = *lane_pose_opt;
    EXPECT_NEAR(lane_pose.s, pose.s, 1e-9);
    EXPECT_NEAR(lane_pose.t, pose.t, 1e-9);
    EXPECT_NEAR(lane_pose.h, pose.h, 1e-9);
    EXPECT_EQ(lane_pose.lane, pose.lane);
  }

  // Act & Assert 2: Query right lane (lane0, original ID -1) at s = 10.0, t = -0.5, h = 0.0
  {
    strada::cpm::LanePose pose;
    pose.s = 10.0;
    pose.t = -0.5;
    pose.h = 0.0;
    pose.heading = 0.0;
    pose.pitch = 0.0;
    pose.roll = 0.0;
    pose.road = strada::cpm::RoadId{0};
    pose.lane = lane0;

    auto road_pose = lane_network.LaneToRoad(pose, ctx);
    EXPECT_NEAR(road_pose.s, 10.0, 1e-9);
    EXPECT_NEAR(road_pose.t, 873.5, 1e-9);
    EXPECT_NEAR(road_pose.h, 0.0, 1e-9);
    EXPECT_EQ(road_pose.road, strada::cpm::RoadId{0});

    auto lane_pose_opt = lane_network.RoadToLane(road_pose, ctx);
    ASSERT_TRUE(lane_pose_opt.has_value());
    auto lane_pose = *lane_pose_opt;
    EXPECT_NEAR(lane_pose.s, pose.s, 1e-9);
    EXPECT_NEAR(lane_pose.t, pose.t, 1e-9);
    EXPECT_NEAR(lane_pose.h, pose.h, 1e-9);
    EXPECT_EQ(lane_pose.lane, pose.lane);
  }
}

}  // namespace strada::cpm
