// SPDX-License-Identifier: BSL-1.0

#include <numbers>
#include <strada/cpm/query_context.hpp>
#include <strada/vis/geometry_batcher.hpp>

#include "../cpm/rotation.hpp"

namespace strada::vis {

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
  cpm::QueryContext query_ctx;

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

              cpm::InertialPose inertial_pose = cpm.RoadToInertial(corner_pose, query_ctx);
              world_corners.push_back(Vertex{.x = static_cast<float>(inertial_pose.x),
                                             .y = static_cast<float>(inertial_pose.y),
                                             .z = static_cast<float>(inertial_pose.z),
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

  // 5. Batch Road Signals and Signal References
  for (const auto& road : map.roads) {
    auto opt_road_id = cpm.RoadIdFromString(road.id);
    if (!opt_road_id) {
      continue;
    }
    const cpm::RoadId road_id = *opt_road_id;

    for (const auto& signal : road.signals) {
      cpm::RoadPose pose_bottom;
      pose_bottom.road = road_id;
      pose_bottom.s = signal.s;
      pose_bottom.t = signal.t;
      pose_bottom.h = 0.0;
      pose_bottom.heading = 0.0;
      pose_bottom.pitch = 0.0;
      pose_bottom.roll = 0.0;

      cpm::RoadPose pose_top;
      pose_top.road = road_id;
      pose_top.s = signal.s;
      pose_top.t = signal.t;
      pose_top.h = signal.z_offset;
      pose_top.heading = signal.h_offset;
      pose_top.pitch = signal.pitch;
      pose_top.roll = signal.roll;

      cpm::InertialPose ip_bottom = cpm.RoadToInertial(pose_bottom, query_ctx);
      cpm::InertialPose ip_top = cpm.RoadToInertial(pose_top, query_ctx);

      batched.signal_line_vertices.push_back(Vertex{.x = static_cast<float>(ip_bottom.x),
                                                    .y = static_cast<float>(ip_bottom.y),
                                                    .z = static_cast<float>(ip_bottom.z),
                                                    .r = kSignalColor.r,
                                                    .g = kSignalColor.g,
                                                    .b = kSignalColor.b});
      batched.signal_line_vertices.push_back(Vertex{.x = static_cast<float>(ip_top.x),
                                                    .y = static_cast<float>(ip_top.y),
                                                    .z = static_cast<float>(ip_top.z),
                                                    .r = kSignalColor.r,
                                                    .g = kSignalColor.g,
                                                    .b = kSignalColor.b});

      auto r_obj = cpm::Rotation::FromEuler(ip_top.heading, ip_top.pitch, ip_top.roll);
      if (signal.width > 0.0 && signal.height > 0.0) {
        double half_w = signal.width * 0.5;
        double half_h = signal.height * 0.5;
        std::array<std::array<double, 3>, 4> local_corners = {
            {{0.0, -half_w, -half_h}, {0.0, half_w, -half_h}, {0.0, half_w, half_h}, {0.0, -half_w, half_h}}};

        std::array<Vertex, 4> world_corners;
        for (std::size_t i = 0; i < 4; ++i) {
          auto local_pos = r_obj.Transform(local_corners[i][0], local_corners[i][1], local_corners[i][2]);
          world_corners[i] = Vertex{.x = static_cast<float>(ip_top.x + local_pos[0]),
                                    .y = static_cast<float>(ip_top.y + local_pos[1]),
                                    .z = static_cast<float>(ip_top.z + local_pos[2]),
                                    .r = kSignalColor.r,
                                    .g = kSignalColor.g,
                                    .b = kSignalColor.b};
        }

        for (std::size_t i = 0; i < 4; ++i) {
          batched.signal_line_vertices.push_back(world_corners[i]);
          batched.signal_line_vertices.push_back(world_corners[(i + 1) % 4]);
        }
      } else {
        double radius = (signal.width > 0.0) ? signal.width * 0.5 : 0.25;
        constexpr std::size_t segments = 12;
        std::array<Vertex, segments> world_circle;
        for (std::size_t i = 0; i < segments; ++i) {
          double theta = 2.0 * std::numbers::pi * static_cast<double>(i) / static_cast<double>(segments);
          auto local_pos = r_obj.Transform(0.0, radius * std::cos(theta), radius * std::sin(theta));
          world_circle[i] = Vertex{.x = static_cast<float>(ip_top.x + local_pos[0]),
                                   .y = static_cast<float>(ip_top.y + local_pos[1]),
                                   .z = static_cast<float>(ip_top.z + local_pos[2]),
                                   .r = kSignalColor.r,
                                   .g = kSignalColor.g,
                                   .b = kSignalColor.b};
        }

        for (std::size_t i = 0; i < segments; ++i) {
          batched.signal_line_vertices.push_back(world_circle[i]);
          batched.signal_line_vertices.push_back(world_circle[(i + 1) % segments]);
        }
      }
    }

    for (const auto& sig_ref : road.signal_references) {
      cpm::RoadPose pose_bottom;
      pose_bottom.road = road_id;
      pose_bottom.s = sig_ref.s;
      pose_bottom.t = sig_ref.t;
      pose_bottom.h = 0.0;
      pose_bottom.heading = 0.0;
      pose_bottom.pitch = 0.0;
      pose_bottom.roll = 0.0;

      cpm::RoadPose pose_top;
      pose_top.road = road_id;
      pose_top.s = sig_ref.s;
      pose_top.t = sig_ref.t;
      pose_top.h = sig_ref.z_offset;
      pose_top.heading = 0.0;
      pose_top.pitch = 0.0;
      pose_top.roll = 0.0;

      cpm::InertialPose ip_bottom = cpm.RoadToInertial(pose_bottom, query_ctx);
      cpm::InertialPose ip_top = cpm.RoadToInertial(pose_top, query_ctx);

      batched.signal_line_vertices.push_back(Vertex{.x = static_cast<float>(ip_bottom.x),
                                                    .y = static_cast<float>(ip_bottom.y),
                                                    .z = static_cast<float>(ip_bottom.z),
                                                    .r = kSignalColor.r,
                                                    .g = kSignalColor.g,
                                                    .b = kSignalColor.b});
      batched.signal_line_vertices.push_back(Vertex{.x = static_cast<float>(ip_top.x),
                                                    .y = static_cast<float>(ip_top.y),
                                                    .z = static_cast<float>(ip_top.z),
                                                    .r = kSignalColor.r,
                                                    .g = kSignalColor.g,
                                                    .b = kSignalColor.b});

      auto r_obj = cpm::Rotation::FromEuler(ip_top.heading, ip_top.pitch, ip_top.roll);
      double radius = 0.25;
      constexpr std::size_t segments = 12;
      std::array<Vertex, segments> world_circle;
      for (std::size_t i = 0; i < segments; ++i) {
        double theta = 2.0 * std::numbers::pi * static_cast<double>(i) / static_cast<double>(segments);
        auto local_pos = r_obj.Transform(0.0, radius * std::cos(theta), radius * std::sin(theta));
        world_circle[i] = Vertex{.x = static_cast<float>(ip_top.x + local_pos[0]),
                                 .y = static_cast<float>(ip_top.y + local_pos[1]),
                                 .z = static_cast<float>(ip_top.z + local_pos[2]),
                                 .r = kSignalColor.r,
                                 .g = kSignalColor.g,
                                 .b = kSignalColor.b};
      }

      for (std::size_t i = 0; i < segments; ++i) {
        batched.signal_line_vertices.push_back(world_circle[i]);
        batched.signal_line_vertices.push_back(world_circle[(i + 1) % segments]);
      }
    }
  }

  return batched;
}

}  // namespace strada::vis
