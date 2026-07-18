#include <gtest/gtest.h>

#include <filesystem>
#include <strada/cpm/coordinate.hpp>
#include <strada/cpm/ids.hpp>
#include <strada/cpm/query_context.hpp>
#include <strada/strada.hpp>
#include <string_view>

namespace {

constexpr auto kMinimalXml = std::string_view{R"(<?xml version="1.0" standalone="yes"?>
<OpenDRIVE>
  <header revMajor="1" revMinor="9" name="Test Map" version="1.0" date="2026-06-14T09:00:00"/>
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
  </road>
</OpenDRIVE>)"};

}  // namespace

TEST(StradaTest, ConstructsFromFileAndExposesLayers) {
  // Arrange
  const auto data_dir = std::filesystem::path{STRADA_TEST_DATA_DIR};
  const auto file_path = data_dir / "roads.xodr";

  // Act
  const auto strada = strada::Strada{file_path};

  // Assert
  EXPECT_EQ(strada.AbstractSyntaxTree().roads.size(), 2);
  EXPECT_EQ(strada.CompiledPhysicsModel().RoadCount(), 2);
  EXPECT_TRUE(strada.Graph().HasRoad("1"));
  EXPECT_TRUE(strada.Graph().HasRoad("2"));

  // Tessellator is enabled by default (chord_error = 0.5)
  const auto tess_opt = strada.Tessellator();
  ASSERT_TRUE(tess_opt.has_value());
  const auto& tess = tess_opt->get();
  EXPECT_FALSE(tess.Polylines().empty());
}

TEST(StradaTest, ConstructsFromStringAndExposesLayers) {
  // Act
  const auto strada = strada::Strada{kMinimalXml};

  // Assert
  EXPECT_EQ(strada.AbstractSyntaxTree().roads.size(), 1);
  EXPECT_EQ(strada.CompiledPhysicsModel().RoadCount(), 1);
  EXPECT_TRUE(strada.Graph().HasRoad("1"));

  const auto tess_opt = strada.Tessellator();
  ASSERT_TRUE(tess_opt.has_value());
}

TEST(StradaTest, TessellatorDisabledWhenChordErrorIsNullopt) {
  // Arrange
  auto options = strada::Strada::Options{.chord_error = std::nullopt};

  // Act
  const auto strada = strada::Strada{kMinimalXml, options};

  // Assert
  EXPECT_EQ(strada.AbstractSyntaxTree().roads.size(), 1);
  EXPECT_EQ(strada.CompiledPhysicsModel().RoadCount(), 1);
  EXPECT_FALSE(strada.Tessellator().has_value());
}

TEST(StradaTest, CopySemanticsProduceIndependentInstances) {
  // Arrange
  const auto strada_orig = strada::Strada{kMinimalXml};

  // Act - copy construct and copy assign
  const auto strada_copy = strada::Strada{strada_orig};
  auto strada_assign = strada::Strada{kMinimalXml};
  strada_assign = strada_orig;

  // Assert - all layers are accessible and equivalent
  EXPECT_EQ(strada_orig.AbstractSyntaxTree().roads.size(), strada_copy.AbstractSyntaxTree().roads.size());
  EXPECT_EQ(strada_orig.AbstractSyntaxTree().roads.size(), strada_assign.AbstractSyntaxTree().roads.size());
  EXPECT_EQ(strada_orig.CompiledPhysicsModel().RoadCount(), strada_copy.CompiledPhysicsModel().RoadCount());
  EXPECT_EQ(strada_orig.CompiledPhysicsModel().RoadCount(), strada_assign.CompiledPhysicsModel().RoadCount());

  // Tessellator is present on all copies
  ASSERT_TRUE(strada_orig.Tessellator().has_value());
  ASSERT_TRUE(strada_copy.Tessellator().has_value());
  ASSERT_TRUE(strada_assign.Tessellator().has_value());

  // CPM queries produce identical results
  const auto& cpm = strada_orig.CompiledPhysicsModel();
  const auto& cpm_copy = strada_copy.CompiledPhysicsModel();
  const auto road_id_opt = cpm.RoadIdFromString("1");
  ASSERT_TRUE(road_id_opt.has_value());
  const auto road_id = *road_id_opt;

  auto pose = strada::cpm::RoadPose{};
  pose.s = 10.0;
  pose.t = 0.0;
  pose.h = 0.0;
  pose.heading = 0.0;
  pose.pitch = 0.0;
  pose.roll = 0.0;
  pose.road = road_id;

  auto ctx_orig = strada::cpm::QueryContext{};
  auto ctx_copy = strada::cpm::QueryContext{};

  const auto ip_orig = cpm.RoadToInertial(pose, ctx_orig);
  const auto ip_copy = cpm_copy.RoadToInertial(pose, ctx_copy);

  EXPECT_DOUBLE_EQ(ip_orig.x, ip_copy.x);
  EXPECT_DOUBLE_EQ(ip_orig.y, ip_copy.y);
  EXPECT_DOUBLE_EQ(ip_orig.z, ip_copy.z);
}

TEST(StradaTest, MoveSemanticsPreserveLayers) {
  // Arrange
  auto strada_src = strada::Strada{kMinimalXml};
  const auto expected_road_count = strada_src.CompiledPhysicsModel().RoadCount();

  // Act - move construct and move assign
  auto strada_dst = strada::Strada{std::move(strada_src)};
  auto strada_assign = strada::Strada{kMinimalXml};
  strada_assign = std::move(strada_dst);

  // Assert
  EXPECT_EQ(strada_assign.CompiledPhysicsModel().RoadCount(), expected_road_count);
  EXPECT_TRUE(strada_assign.Tessellator().has_value());
}
