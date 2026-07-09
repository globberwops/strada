// SPDX-License-Identifier: BSL-1.0

#include <span>
#include <strada/vis/geometry_batcher.hpp>

namespace strada::vis {

namespace {

void BatchOutlines(std::span<const std::vector<tess::Vertex>> outlines, const Color& color,
                   std::vector<Vertex>& output_vertices) {
  for (const auto& outline : outlines) {
    if (outline.size() < 2) {
      continue;
    }
    for (std::size_t i = 0; i < outline.size() - 1; ++i) {
      const auto& p0 = outline[i];
      const auto& p1 = outline[i + 1];
      output_vertices.push_back(Vertex{.x = p0.x, .y = p0.y, .z = p0.z, .r = color.r, .g = color.g, .b = color.b});
      output_vertices.push_back(Vertex{.x = p1.x, .y = p1.y, .z = p1.z, .r = color.r, .g = color.g, .b = color.b});
    }
  }
}

}  // namespace

auto BatchMapGeometry(const tess::Tessellator& tess, const ast::AbstractSyntaxTree& map,
                      const cpm::CompiledPhysicsModel& cpm) -> BatchedGeometry {
  BatchedGeometry batched;

  // 1. Batch meshes for GL_TRIANGLES
  std::uint32_t vertex_offset = 0;
  for (const auto& mesh : tess.Meshes()) {
    const Color lane_color = GetLaneColor(mesh.lane_type, mesh.original_lane_id);

    auto index_start = static_cast<std::uint32_t>(batched.triangle_indices.size());
    auto index_count = static_cast<std::uint32_t>(mesh.indices.size());

    for (const auto& v : mesh.vertices) {
      batched.triangle_vertices.push_back(
          Vertex{.x = v.x, .y = v.y, .z = v.z, .r = lane_color.r, .g = lane_color.g, .b = lane_color.b});
    }

    for (const std::uint32_t idx : mesh.indices) {
      batched.triangle_indices.push_back(idx + vertex_offset);
    }

    batched.mesh_ranges.push_back(MeshRange{.road_id = mesh.road_id,
                                            .lane_id = mesh.lane_id,
                                            .lane_type = mesh.lane_type,
                                            .index_start = index_start,
                                            .index_count = index_count});

    vertex_offset += static_cast<std::uint32_t>(mesh.vertices.size());
  }

  // 2. Batch polylines for GL_LINES
  for (const auto& poly : tess.Polylines()) {
    if (!poly.is_reference_line) {
      continue;
    }

    constexpr Color line_color = kReferenceLineColor;

    if (poly.vertices.size() < 2) {
      continue;
    }

    for (std::size_t i = 0; i < poly.vertices.size() - 1; ++i) {
      const auto& vertex0 = poly.vertices[i];
      const auto& vertex1 = poly.vertices[i + 1];

      batched.line_vertices.push_back(Vertex{
          .x = vertex0.x, .y = vertex0.y, .z = vertex0.z, .r = line_color.r, .g = line_color.g, .b = line_color.b});
      batched.line_vertices.push_back(Vertex{
          .x = vertex1.x, .y = vertex1.y, .z = vertex1.z, .r = line_color.r, .g = line_color.g, .b = line_color.b});
    }
  }

  // 3. Batch junction boundaries
  for (const auto& boundary_geom : tess.JunctionBoundaries()) {
    auto current_offset = static_cast<std::uint32_t>(batched.boundary_triangle_vertices.size());
    for (const auto& v : boundary_geom.vertices) {
      batched.boundary_triangle_vertices.push_back(Vertex{.x = v.x,
                                                          .y = v.y,
                                                          .z = v.z,
                                                          .r = kJunctionBoundaryColor.r,
                                                          .g = kJunctionBoundaryColor.g,
                                                          .b = kJunctionBoundaryColor.b});
    }
    for (const std::uint32_t idx : boundary_geom.indices) {
      batched.boundary_triangle_indices.push_back(idx + current_offset);
    }
  }

  // 4. Batch Road Objects
  for (const auto& obj : tess.Objects()) {
    BatchOutlines(obj.outlines, kObjectColor, batched.object_line_vertices);
  }

  // 5. Batch Road Signals and Signal References
  for (const auto& sig : tess.Signals()) {
    BatchOutlines(sig.outlines, kSignalColor, batched.signal_line_vertices);
  }

  return batched;
}

}  // namespace strada::vis
