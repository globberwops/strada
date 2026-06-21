// SPDX-License-Identifier: BSL-1.0

#include <strada/vis/geometry_batcher.hpp>

namespace strada::vis {

auto GetLaneColor(const std::string& lane_type) noexcept -> Color {
  if (lane_type == "driving") {
    return Color{0.2f, 0.25f, 0.3f};
  }
  if (lane_type == "sidewalk") {
    return Color{0.4f, 0.45f, 0.5f};
  }
  if (lane_type == "shoulder") {
    return Color{0.35f, 0.3f, 0.25f};
  }
  return Color{0.25f, 0.3f, 0.25f};
}

auto BatchMapGeometry(const tess::Tessellator& tess) -> BatchedGeometry {
  BatchedGeometry batched;

  // 1. Batch meshes for GL_TRIANGLES
  std::uint32_t vertex_offset = 0;
  for (const auto& mesh : tess.Meshes()) {
    Color lane_color = GetLaneColor(mesh.lane_type);

    std::uint32_t index_start = static_cast<std::uint32_t>(batched.triangle_indices.size());
    std::uint32_t index_count = static_cast<std::uint32_t>(mesh.indices.size());

    for (const auto& v : mesh.vertices) {
      batched.triangle_vertices.push_back(
          Vertex{.x = v.x, .y = v.y, .z = v.z, .r = lane_color.r, .g = lane_color.g, .b = lane_color.b});
    }

    for (std::uint32_t idx : mesh.indices) {
      batched.triangle_indices.push_back(idx + vertex_offset);
    }

    batched.mesh_ranges.push_back(MeshRange{
        .road_id = mesh.road_id, .lane_id = mesh.lane_id, .index_start = index_start, .index_count = index_count});

    vertex_offset += static_cast<std::uint32_t>(mesh.vertices.size());
  }

  // 2. Batch polylines for GL_LINES
  for (const auto& poly : tess.Polylines()) {
    Color line_color = poly.is_reference_line ? Color{245.0f / 255.0f, 197.0f / 255.0f, 61.0f / 255.0f}
                                              : Color{230.0f / 255.0f, 230.0f / 255.0f, 230.0f / 255.0f};

    if (poly.vertices.size() < 2) {
      continue;
    }

    for (size_t i = 0; i < poly.vertices.size() - 1; ++i) {
      const auto& v0 = poly.vertices[i];
      const auto& v1 = poly.vertices[i + 1];

      batched.line_vertices.push_back(
          Vertex{.x = v0.x, .y = v0.y, .z = v0.z, .r = line_color.r, .g = line_color.g, .b = line_color.b});
      batched.line_vertices.push_back(
          Vertex{.x = v1.x, .y = v1.y, .z = v1.z, .r = line_color.r, .g = line_color.g, .b = line_color.b});
    }
  }

  return batched;
}

}  // namespace strada::vis
