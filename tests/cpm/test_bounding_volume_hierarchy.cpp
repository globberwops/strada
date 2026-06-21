#include <gtest/gtest.h>

#include <algorithm>
#include <strada/cpm/bounding_volume_hierarchy.hpp>

namespace strada::cpm {

TEST(BoundingVolumeHierarchyTest, SingleNodeConstruction) {
  // Arrange
  std::vector<uint32_t> prim_indices = {0};
  std::vector<BoundingVolumeHierarchy::PrimitiveInfo> temp_primitives = {{.road_idx = 0, .segment_idx = 0}};
  std::vector<Aabb> temp_aabbs = {{.min_x = 10.0, .min_y = 20.0, .max_x = 30.0, .max_y = 40.0}};

  // Act
  auto bounding_volume_hierarchy = BoundingVolumeHierarchy::Build(prim_indices, temp_primitives, temp_aabbs);

  // Assert
  ASSERT_EQ(bounding_volume_hierarchy.Nodes().size(), 1);
  const auto& root = bounding_volume_hierarchy.Nodes()[0];
  EXPECT_DOUBLE_EQ(root.min_x, 10.0);
  EXPECT_DOUBLE_EQ(root.min_y, 20.0);
  EXPECT_DOUBLE_EQ(root.max_x, 30.0);
  EXPECT_DOUBLE_EQ(root.max_y, 40.0);
  EXPECT_EQ(root.left, 0);            // prim_start index
  EXPECT_EQ(root.right, 0x80000001);  // leaf flag + prim_count (1)
}

TEST(BoundingVolumeHierarchyTest, RecursiveSplitting) {
  // Arrange
  std::vector<uint32_t> prim_indices = {0, 1, 2, 3, 4};
  std::vector<BoundingVolumeHierarchy::PrimitiveInfo> temp_primitives = {{.road_idx = 0, .segment_idx = 0},
                                                                         {.road_idx = 1, .segment_idx = 0},
                                                                         {.road_idx = 2, .segment_idx = 0},
                                                                         {.road_idx = 3, .segment_idx = 0},
                                                                         {.road_idx = 4, .segment_idx = 0}};

  std::vector<Aabb> temp_aabbs = {{.min_x = 0.0, .min_y = 0.0, .max_x = 1.0, .max_y = 1.0},
                                  {.min_x = 10.0, .min_y = 0.0, .max_x = 11.0, .max_y = 1.0},
                                  {.min_x = 20.0, .min_y = 0.0, .max_x = 21.0, .max_y = 1.0},
                                  {.min_x = 30.0, .min_y = 0.0, .max_x = 31.0, .max_y = 1.0},
                                  {.min_x = 40.0, .min_y = 0.0, .max_x = 41.0, .max_y = 1.0}};

  // Act
  auto bounding_volume_hierarchy = BoundingVolumeHierarchy::Build(prim_indices, temp_primitives, temp_aabbs);

  // Assert
  ASSERT_GT(bounding_volume_hierarchy.Nodes().size(), 1);
  const auto& root = bounding_volume_hierarchy.Nodes()[0];
  EXPECT_FALSE((root.right & BoundingVolumeHierarchy::kLeafBitMask) != 0);  // Root is internal
  EXPECT_DOUBLE_EQ(root.min_x, 0.0);
  EXPECT_DOUBLE_EQ(root.max_x, 41.0);
}

TEST(BoundingVolumeHierarchyTest, TraversalAndPruning) {
  // Arrange
  std::vector<uint32_t> prim_indices = {0, 1, 2, 3, 4};
  std::vector<BoundingVolumeHierarchy::PrimitiveInfo> temp_primitives = {{.road_idx = 0, .segment_idx = 0},
                                                                         {.road_idx = 1, .segment_idx = 0},
                                                                         {.road_idx = 2, .segment_idx = 0},
                                                                         {.road_idx = 3, .segment_idx = 0},
                                                                         {.road_idx = 4, .segment_idx = 0}};

  std::vector<Aabb> temp_aabbs = {{.min_x = 0.0, .min_y = 0.0, .max_x = 1.0, .max_y = 1.0},
                                  {.min_x = 10.0, .min_y = 0.0, .max_x = 11.0, .max_y = 1.0},
                                  {.min_x = 20.0, .min_y = 0.0, .max_x = 21.0, .max_y = 1.0},
                                  {.min_x = 30.0, .min_y = 0.0, .max_x = 31.0, .max_y = 1.0},
                                  {.min_x = 40.0, .min_y = 0.0, .max_x = 41.0, .max_y = 1.0}};

  auto bounding_volume_hierarchy = BoundingVolumeHierarchy::Build(prim_indices, temp_primitives, temp_aabbs);

  // Act
  std::vector<uint32_t> visited_roads;
  bounding_volume_hierarchy.Query(
      0.5, 0.5,
      [&](const BoundingVolumeHierarchy::PrimitiveInfo& prim, double current_min_dist) -> std::optional<double> {
        visited_roads.push_back(prim.road_idx);
        if (prim.road_idx == 0) {
          return 0.1;
        }
        return std::nullopt;
      });

  // Assert
  EXPECT_TRUE(std::find(visited_roads.begin(), visited_roads.end(), 0) != visited_roads.end());
  EXPECT_TRUE(std::find(visited_roads.begin(), visited_roads.end(), 4) == visited_roads.end());
}

}  // namespace strada::cpm
