// SPDX-License-Identifier: BSL-1.0

#pragma once

#include <cstdint>
#include <strada/ast/abstract_syntax_tree.hpp>
#include <strada/cpm/compiled_physics_model.hpp>
#include <strada/tess/tessellator.hpp>
#include <strada/vis/colors.hpp>
#include <vector>

namespace strada::vis {

/// Vertex layout for batched rendering.
struct Vertex {
  float x{};
  float y{};
  float z{};
  float r{};
  float g{};
  float b{};
};

/// Tracks index offsets and counts in the combined index buffer for rendering individual lanes.
struct MeshRange {
  cpm::RoadId road_id{};
  cpm::LaneId lane_id{};
  ast::LaneType lane_type{ast::LaneType::kNone};
  std::uint32_t index_start{};
  std::uint32_t index_count{};
};

/// Holds pre-batched layout ready to load into OpenGL VBOs/IBOs.
struct BatchedGeometry {
  std::vector<Vertex> triangle_vertices;
  std::vector<std::uint32_t> triangle_indices;
  std::vector<Vertex> line_vertices;
  std::vector<MeshRange> mesh_ranges;
  std::vector<Vertex> boundary_triangle_vertices;
  std::vector<std::uint32_t> boundary_triangle_indices;
  std::vector<Vertex> object_line_vertices;
  std::vector<Vertex> signal_line_vertices;
};

/// Batches all map meshes and polylines into contiguous arrays.
auto BatchMapGeometry(const tess::Tessellator& tess) -> BatchedGeometry;

}  // namespace strada::vis
