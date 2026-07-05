// SPDX-License-Identifier: BSL-1.0

#include <strada/cpm/query_context.hpp>
#include <strada/vis/geometry_batcher.hpp>

#include "../cpm/rotation.hpp"

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

auto BatchMapGeometry(const tess::Tessellator& tess, const ast::AbstractSyntaxTree& map,
                      const cpm::CompiledPhysicsModel& cpm) -> BatchedGeometry {
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

    auto line_color = Color{.r = 1.0F, .g = 0.0F, .b = 0.0F};

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

  // 4. Batch Road Objects
  cpm::QueryContext query_ctx;
  const Color kObjectColor{.r = 1.0F, .g = 145.0F / 255.0F, .b = 0.0F};

  for (const auto& road : map.roads) {
    auto opt_road_id = cpm.RoadIdFromString(road.id);
    if (!opt_road_id) {
      continue;
    }
    const cpm::RoadId road_id = *opt_road_id;

    for (const auto& object : road.objects) {
      bool has_outlines = false;
      for (const auto& outline : object.outlines) {
        if (!outline.corners_local.empty() || !outline.corners_road.empty()) {
          has_outlines = true;
          break;
        }
      }

      if (has_outlines) {
        for (const auto& outline : object.outlines) {
          if (!outline.corners_local.empty()) {
            const std::size_t num_corners = outline.corners_local.size();
            std::vector<Vertex> world_corners;
            world_corners.reserve(num_corners);

            cpm::RoadPose obj_pose;
            obj_pose.road = road_id;
            obj_pose.s = object.s;
            obj_pose.t = object.t;
            obj_pose.h = object.z_offset;
            obj_pose.heading = object.hdg;
            obj_pose.pitch = object.pitch;
            obj_pose.roll = object.roll;

            cpm::InertialPose ip_obj = cpm.RoadToInertial(obj_pose, query_ctx);
            auto r_obj = cpm::Rotation::FromEuler(ip_obj.heading, ip_obj.pitch, ip_obj.roll);

            for (const auto& corner : outline.corners_local) {
              auto local_pos = r_obj.Transform(corner.u, corner.v, corner.z);
              world_corners.push_back(Vertex{.x = static_cast<float>(ip_obj.x + local_pos[0]),
                                             .y = static_cast<float>(ip_obj.y + local_pos[1]),
                                             .z = static_cast<float>(ip_obj.z + local_pos[2]),
                                             .r = kObjectColor.r,
                                             .g = kObjectColor.g,
                                             .b = kObjectColor.b});
            }

            for (std::size_t i = 0; i < num_corners - 1; ++i) {
              batched.object_line_vertices.push_back(world_corners[i]);
              batched.object_line_vertices.push_back(world_corners[i + 1]);
            }
            if (outline.closed && num_corners > 2) {
              batched.object_line_vertices.push_back(world_corners.back());
              batched.object_line_vertices.push_back(world_corners.front());
            }
          } else if (!outline.corners_road.empty()) {
            const std::size_t num_corners = outline.corners_road.size();
            std::vector<Vertex> world_corners;
            world_corners.reserve(num_corners);

            for (const auto& corner : outline.corners_road) {
              cpm::RoadPose corner_pose;
              corner_pose.road = road_id;
              corner_pose.s = corner.s;
              corner_pose.t = corner.t;
              corner_pose.h = corner.dz;
              corner_pose.heading = 0.0;
              corner_pose.pitch = 0.0;
              corner_pose.roll = 0.0;

              cpm::InertialPose ip = cpm.RoadToInertial(corner_pose, query_ctx);
              world_corners.push_back(Vertex{.x = static_cast<float>(ip.x),
                                             .y = static_cast<float>(ip.y),
                                             .z = static_cast<float>(ip.z),
                                             .r = kObjectColor.r,
                                             .g = kObjectColor.g,
                                             .b = kObjectColor.b});
            }

            for (std::size_t i = 0; i < num_corners - 1; ++i) {
              batched.object_line_vertices.push_back(world_corners[i]);
              batched.object_line_vertices.push_back(world_corners[i + 1]);
            }
            if (outline.closed && num_corners > 2) {
              batched.object_line_vertices.push_back(world_corners.back());
              batched.object_line_vertices.push_back(world_corners.front());
            }
          }
        }
      } else if (object.length > 0.0 && object.width > 0.0) {
        double half_l = object.length * 0.5;
        double half_w = object.width * 0.5;

        cpm::RoadPose obj_pose;
        obj_pose.road = road_id;
        obj_pose.s = object.s;
        obj_pose.t = object.t;
        obj_pose.h = object.z_offset;
        obj_pose.heading = object.hdg;
        obj_pose.pitch = object.pitch;
        obj_pose.roll = object.roll;

        cpm::InertialPose ip_obj = cpm.RoadToInertial(obj_pose, query_ctx);
        auto r_obj = cpm::Rotation::FromEuler(ip_obj.heading, ip_obj.pitch, ip_obj.roll);

        std::array<std::pair<double, double>, 4> local_pts = {
            {{half_l, half_w}, {half_l, -half_w}, {-half_l, -half_w}, {-half_l, half_w}}};

        std::array<Vertex, 4> world_pts;
        for (std::size_t i = 0; i < 4; ++i) {
          auto local_pos = r_obj.Transform(local_pts[i].first, local_pts[i].second, 0.0);
          world_pts[i] = Vertex{.x = static_cast<float>(ip_obj.x + local_pos[0]),
                                .y = static_cast<float>(ip_obj.y + local_pos[1]),
                                .z = static_cast<float>(ip_obj.z + local_pos[2]),
                                .r = kObjectColor.r,
                                .g = kObjectColor.g,
                                .b = kObjectColor.b};
        }

        batched.object_line_vertices.push_back(world_pts[0]);
        batched.object_line_vertices.push_back(world_pts[1]);
        batched.object_line_vertices.push_back(world_pts[1]);
        batched.object_line_vertices.push_back(world_pts[2]);
        batched.object_line_vertices.push_back(world_pts[2]);
        batched.object_line_vertices.push_back(world_pts[3]);
        batched.object_line_vertices.push_back(world_pts[3]);
        batched.object_line_vertices.push_back(world_pts[0]);
      } else {
        cpm::RoadPose obj_pose;
        obj_pose.road = road_id;
        obj_pose.s = object.s;
        obj_pose.t = object.t;
        obj_pose.h = object.z_offset;
        obj_pose.heading = object.hdg;
        obj_pose.pitch = object.pitch;
        obj_pose.roll = object.roll;

        cpm::InertialPose ip_obj = cpm.RoadToInertial(obj_pose, query_ctx);
        auto r_obj = cpm::Rotation::FromEuler(ip_obj.heading, ip_obj.pitch, ip_obj.roll);

        auto local1_a = r_obj.Transform(0.0, -0.25, 0.0);
        auto local1_b = r_obj.Transform(0.0, 0.25, 0.0);
        auto local2_a = r_obj.Transform(-0.25, 0.0, 0.0);
        auto local2_b = r_obj.Transform(0.25, 0.0, 0.0);

        batched.object_line_vertices.push_back(Vertex{.x = static_cast<float>(ip_obj.x + local1_a[0]),
                                                      .y = static_cast<float>(ip_obj.y + local1_a[1]),
                                                      .z = static_cast<float>(ip_obj.z + local1_a[2]),
                                                      .r = kObjectColor.r,
                                                      .g = kObjectColor.g,
                                                      .b = kObjectColor.b});
        batched.object_line_vertices.push_back(Vertex{.x = static_cast<float>(ip_obj.x + local1_b[0]),
                                                      .y = static_cast<float>(ip_obj.y + local1_b[1]),
                                                      .z = static_cast<float>(ip_obj.z + local1_b[2]),
                                                      .r = kObjectColor.r,
                                                      .g = kObjectColor.g,
                                                      .b = kObjectColor.b});

        batched.object_line_vertices.push_back(Vertex{.x = static_cast<float>(ip_obj.x + local2_a[0]),
                                                      .y = static_cast<float>(ip_obj.y + local2_a[1]),
                                                      .z = static_cast<float>(ip_obj.z + local2_a[2]),
                                                      .r = kObjectColor.r,
                                                      .g = kObjectColor.g,
                                                      .b = kObjectColor.b});
        batched.object_line_vertices.push_back(Vertex{.x = static_cast<float>(ip_obj.x + local2_b[0]),
                                                      .y = static_cast<float>(ip_obj.y + local2_b[1]),
                                                      .z = static_cast<float>(ip_obj.z + local2_b[2]),
                                                      .r = kObjectColor.r,
                                                      .g = kObjectColor.g,
                                                      .b = kObjectColor.b});
      }
    }
  }

  return batched;
}

}  // namespace strada::vis
