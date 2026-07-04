// SPDX-License-Identifier: BSL-1.0

#include <strada/vis/geometry_batcher.hpp>

namespace strada::vis {

auto GetLaneColor(const std::string& lane_type) noexcept -> Color {
  if (lane_type == "driving") {
    return Color{.r = 0.2F, .g = 0.25F, .b = 0.3F};
  }
  if (lane_type == "sidewalk") {
    return Color{.r = 0.4F, .g = 0.45F, .b = 0.5F};
  }
  if (lane_type == "shoulder") {
    return Color{.r = 0.35F, .g = 0.3F, .b = 0.25F};
  }
  return Color{.r = 0.25F, .g = 0.3F, .b = 0.25F};
}

auto BatchMapGeometry(const tess::Tessellator& tess) -> BatchedGeometry {
  BatchedGeometry batched;

  // 1. Batch meshes for GL_TRIANGLES
  std::uint32_t vertex_offset = 0;
  for (const auto& mesh : tess.Meshes()) {
    const Color lane_color = GetLaneColor(mesh.lane_type);

    auto index_start = static_cast<std::uint32_t>(batched.triangle_indices.size());
    auto index_count = static_cast<std::uint32_t>(mesh.indices.size());

    for (const auto& v : mesh.vertices) {
      batched.triangle_vertices.push_back(
          Vertex{.x = v.x, .y = v.y, .z = v.z, .r = lane_color.r, .g = lane_color.g, .b = lane_color.b});
    }

    for (const std::uint32_t idx : mesh.indices) {
      batched.triangle_indices.push_back(idx + vertex_offset);
    }

    batched.mesh_ranges.push_back(MeshRange{
        .road_id = mesh.road_id, .lane_id = mesh.lane_id, .index_start = index_start, .index_count = index_count});

    vertex_offset += static_cast<std::uint32_t>(mesh.vertices.size());
  }

  // 2. Batch polylines for GL_LINES
  for (const auto& poly : tess.Polylines()) {
    if (!poly.is_reference_line) {
      continue;
    }

    auto line_color = Color{.r = 245.0F / 255.0F, .g = 197.0F / 255.0F, .b = 61.0F / 255.0F};

    if (poly.vertices.size() < 2) {
      continue;
    }

    for (std::size_t i = 0; i < poly.vertices.size() - 1; ++i) {
      const auto& v0 = poly.vertices[i];
      const auto& v1 = poly.vertices[i + 1];

      batched.line_vertices.push_back(
          Vertex{.x = v0.x, .y = v0.y, .z = v0.z, .r = line_color.r, .g = line_color.g, .b = line_color.b});
      batched.line_vertices.push_back(
          Vertex{.x = v1.x, .y = v1.y, .z = v1.z, .r = line_color.r, .g = line_color.g, .b = line_color.b});
    }
  }

  // 3. Batch junction boundaries
  for (const auto& boundary_geom : tess.JunctionBoundaries()) {
    auto current_offset = static_cast<std::uint32_t>(batched.boundary_triangle_vertices.size());
    for (const auto& v : boundary_geom.vertices) {
      batched.boundary_triangle_vertices.push_back(
          Vertex{.x = v.x, .y = v.y, .z = v.z, .r = 245.0F / 255.0F, .g = 197.0F / 255.0F, .b = 61.0F / 255.0F});
    }
    for (const std::uint32_t idx : boundary_geom.indices) {
      batched.boundary_triangle_indices.push_back(idx + current_offset);
    }
  }

  return batched;
}

}  // namespace strada::vis
