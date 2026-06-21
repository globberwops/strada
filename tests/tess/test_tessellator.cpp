// SPDX-License-Identifier: BSL-1.0

#include <gtest/gtest.h>
#include <filesystem>
#include <strada/ast/abstract_syntax_tree.hpp>
#include <strada/parser/parser.hpp>
#include <strada/tess/tessellator.hpp>

#ifndef STRADA_TEST_DATA_DIR
#define STRADA_TEST_DATA_DIR "tests/data"
#endif

namespace strada::tess {

// Helper to construct absolute path to test fixtures
static auto GetTestDataPath(const std::string& filename) -> std::filesystem::path {
  return std::filesystem::path(STRADA_TEST_DATA_DIR) / filename;
}

TEST(TessellatorTest, EmptyMap) {
  // Arrange: empty AST
  ast::AbstractSyntaxTree map;

  // Act: tessellate
  auto tess = BuildTessellator(map, 0.5);

  // Assert: no meshes or polylines
  EXPECT_TRUE(tess.Meshes().empty());
  EXPECT_TRUE(tess.Polylines().empty());
}

TEST(TessellatorTest, StraightRoadReferenceLine) {
  // Arrange: parse roads.xodr containing two straight line roads
  auto map_path = GetTestDataPath("roads.xodr");
  auto map = parser::ParseFile(map_path);

  // Act: tessellate with 0.5m chord error
  auto tess = BuildTessellator(map, 0.5);

  // Assert: we expect 2 reference lines (one per road)
  // Let's filter polylines representing reference lines
  std::vector<Polyline> ref_lines;
  for (const auto& poly : tess.Polylines()) {
    if (poly.is_reference_line) {
      ref_lines.push_back(poly);
    }
  }

  ASSERT_EQ(ref_lines.size(), 2);

  // Road 1 (length 10.0) reference line properties
  const auto& road1_ref = ref_lines[0];
  EXPECT_EQ(road1_ref.road_id, static_cast<cpm::RoadId>(0));
  EXPECT_EQ(road1_ref.marking_type, "solid");
  ASSERT_GE(road1_ref.vertices.size(), 2);

  // First point should be at (0, 0, 0)
  EXPECT_NEAR(road1_ref.vertices.front().x, 0.0f, 1e-3f);
  EXPECT_NEAR(road1_ref.vertices.front().y, 0.0f, 1e-3f);
  EXPECT_NEAR(road1_ref.vertices.front().z, 0.0f, 1e-3f);

  // Last point should be at (10, 0, 0)
  EXPECT_NEAR(road1_ref.vertices.back().x, 10.0f, 1e-3f);
  EXPECT_NEAR(road1_ref.vertices.back().y, 0.0f, 1e-3f);
  EXPECT_NEAR(road1_ref.vertices.back().z, 0.0f, 1e-3f);

  // Road 2 (length 25.5) reference line properties
  const auto& road2_ref = ref_lines[1];
  EXPECT_EQ(road2_ref.road_id, static_cast<cpm::RoadId>(1));
  ASSERT_GE(road2_ref.vertices.size(), 2);

  EXPECT_NEAR(road2_ref.vertices.front().x, 0.0f, 1e-3f);
  EXPECT_NEAR(road2_ref.vertices.front().y, 0.0f, 1e-3f);

  EXPECT_NEAR(road2_ref.vertices.back().x, 25.5f, 1e-3f);
  EXPECT_NEAR(road2_ref.vertices.back().y, 0.0f, 1e-3f);
}

}  // namespace strada::tess
