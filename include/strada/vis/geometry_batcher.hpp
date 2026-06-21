// SPDX-License-Identifier: BSL-1.0

#pragma once

#include <cstdint>
#include <strada/tess/tessellator.hpp>
#include <string>
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

/// Holds pre-batched layout ready to load into OpenGL VBOs/IBOs.
struct BatchedGeometry {
  std::vector<Vertex> triangle_vertices;
  std::vector<std::uint32_t> triangle_indices;
  std::vector<Vertex> line_vertices;
};

/// Curated premium color palette matching dark-mode aesthetics.
struct Color {
  float r{};
  float g{};
  float b{};
};

/// Color lookup helper based on lane type.
auto GetLaneColor(const std::string& lane_type) noexcept -> Color;

/// Batches all map meshes and polylines into contiguous arrays.
auto BatchMapGeometry(const tess::Tessellator& tess) -> BatchedGeometry;

}  // namespace strada::vis
