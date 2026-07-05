// SPDX-License-Identifier: BSL-1.0

#include <strada/vis/geometry_batcher.hpp>

namespace strada::vis {

auto GetLaneColor(ast::LaneType lane_type, int original_lane_id) noexcept -> Color {
  switch (lane_type) {
    case ast::LaneType::kHov:
    case ast::LaneType::kBidirectional:
    case ast::LaneType::kBus:
    case ast::LaneType::kTaxi:
    case ast::LaneType::kRoadWorks:
    case ast::LaneType::kShared:
    case ast::LaneType::kDriving: {
      if (original_lane_id < 0) {
        return Color{.r = 239.0F / 255.0F, .g = 215.0F / 255.0F, .b = 171.0F / 255.0F};
      }
      return Color{.r = 205.0F / 255.0F, .g = 216.0F / 255.0F, .b = 232.0F / 255.0F};
    }
    case ast::LaneType::kBiking:
      return Color{.r = 207.0F / 255.0F, .g = 16.0F / 255.0F, .b = 45.0F / 255.0F};
    case ast::LaneType::kBorder:
      return Color{.r = 165.0F / 255.0F, .g = 94.0F / 255.0F, .b = 55.0F / 255.0F};
    case ast::LaneType::kConnectingRamp:
      return Color{.r = 168.0F / 255.0F, .g = 211.0F / 255.0F, .b = 0.0F / 255.0F};
    case ast::LaneType::kCurb:
      return Color{.r = 151.0F / 255.0F, .g = 120.0F / 255.0F, .b = 211.0F / 255.0F};
    case ast::LaneType::kMwyEntry:
    case ast::LaneType::kEntry:
      return Color{.r = 234.0F / 255.0F, .g = 217.0F / 255.0F, .b = 96.0F / 255.0F};
    case ast::LaneType::kMwyExit:
    case ast::LaneType::kExit:
      return Color{.r = 103.0F / 255.0F, .g = 153.0F / 255.0F, .b = 204.0F / 255.0F};
    case ast::LaneType::kMedian:
      return Color{.r = 124.0F / 255.0F, .g = 84.0F / 255.0F, .b = 71.0F / 255.0F};
    case ast::LaneType::kSpecial1:
    case ast::LaneType::kSpecial2:
    case ast::LaneType::kSpecial3:
    case ast::LaneType::kNone:
      return Color{.r = 147.0F / 255.0F, .g = 149.0F / 255.0F, .b = 152.0F / 255.0F};
    case ast::LaneType::kOffRamp:
      return Color{.r = 35.0F / 255.0F, .g = 121.0F / 255.0F, .b = 185.0F / 255.0F};
    case ast::LaneType::kOnRamp:
      return Color{.r = 255.0F / 255.0F, .g = 212.0F / 255.0F, .b = 2.0F / 255.0F};
    case ast::LaneType::kParking:
      return Color{.r = 98.0F / 255.0F, .g = 38.0F / 255.0F, .b = 158.0F / 255.0F};
    case ast::LaneType::kRail:
      return Color{.r = 56.0F / 255.0F, .g = 43.0F / 255.0F, .b = 178.0F / 255.0F};
    case ast::LaneType::kRestricted:
      return Color{.r = 255.0F / 255.0F, .g = 103.0F / 255.0F, .b = 27.0F / 255.0F};
    case ast::LaneType::kShoulder:
      return Color{.r = 0.0F / 255.0F, .g = 98.0F / 255.0F, .b = 65.0F / 255.0F};
    case ast::LaneType::kSidewalk:
    case ast::LaneType::kWalking:
      return Color{.r = 121.0F / 255.0F, .g = 36.0F / 255.0F, .b = 47.0F / 255.0F};
    case ast::LaneType::kSlipLane:
      return Color{.r = 0.0F / 255.0F, .g = 148.0F / 255.0F, .b = 94.0F / 255.0F};
    case ast::LaneType::kStop:
      return Color{.r = 146.0F / 255.0F, .g = 213.0F / 255.0F, .b = 172.0F / 255.0F};
    case ast::LaneType::kTram:
      return Color{.r = 109.0F / 255.0F, .g = 109.0F / 255.0F, .b = 226.0F / 255.0F};
  }
  return Color{.r = 147.0F / 255.0F, .g = 149.0F / 255.0F, .b = 152.0F / 255.0F};
}

auto BatchMapGeometry(const tess::Tessellator& tess) -> BatchedGeometry {
  BatchedGeometry batched;

  // 1. Batch meshes for GL_TRIANGLES
  std::uint32_t vertex_offset = 0;
  for (const auto& mesh : tess.Meshes()) {
    const Color kLaneColor = GetLaneColor(mesh.lane_type, mesh.original_lane_id);

    auto index_start = static_cast<std::uint32_t>(batched.triangle_indices.size());
    auto index_count = static_cast<std::uint32_t>(mesh.indices.size());

    for (const auto& v : mesh.vertices) {
      batched.triangle_vertices.push_back(
          Vertex{.x = v.x, .y = v.y, .z = v.z, .r = kLaneColor.r, .g = kLaneColor.g, .b = kLaneColor.b});
    }

    for (const std::uint32_t kIdx : mesh.indices) {
      batched.triangle_indices.push_back(kIdx + vertex_offset);
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
    for (const std::uint32_t kIdx : boundary_geom.indices) {
      batched.boundary_triangle_indices.push_back(kIdx + current_offset);
    }
  }

  return batched;
}

}  // namespace strada::vis
