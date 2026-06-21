// SPDX-License-Identifier: BSL-1.0

#include <algorithm>
#include <cmath>
#include <strada/cpm/compiled_physics_model.hpp>
#include <strada/cpm/query_context.hpp>
#include <strada/tess/tessellator.hpp>
#include <variant>

namespace strada::tess {

Tessellator::Tessellator(const ast::AbstractSyntaxTree& map, double chord_error) {
  // 1. Build a temporary CompiledPhysicsModel to evaluate geometries
  auto model = cpm::CompiledPhysicsModel::Build(map);
  cpm::QueryContext ctx;

  // 2. Loop over each road in the AST
  for (size_t road_idx = 0; road_idx < map.roads.size(); ++road_idx) {
    const auto& road = map.roads[road_idx];
    auto road_id = static_cast<cpm::RoadId>(road_idx);
    double road_len = road.length;

    // Determine the sampling stations for this entire road based on its plan-view geometries
    std::vector<double> stations;

    for (const auto& geom : road.plan_view) {
      double geom_s_start = geom.s;
      double geom_length = geom.length;

      // Determine step count based on geometry type and chord error
      size_t num_steps = 10;  // Default fallback

      if (std::holds_alternative<ast::Line>(geom.shape)) {
        num_steps = 1;
      } else if (auto arc_ptr = std::get_if<ast::Arc>(&geom.shape)) {
        double curvature = std::abs(arc_ptr->curvature);
        if (curvature > 1e-6) {
          double radius = 1.0 / curvature;
          double ds = std::sqrt(8.0 * radius * chord_error);
          ds = std::clamp(ds, 0.2, 5.0);  // Clamp step size to reasonable bounds
          num_steps = static_cast<size_t>(std::ceil(geom_length / ds));
        }
      } else {
        // Fallback for spirals and polynomials
        double ds = std::clamp(chord_error * 5.0, 0.5, 2.0);
        num_steps = static_cast<size_t>(std::ceil(geom_length / ds));
      }

      if (num_steps < 1) {
        num_steps = 1;
      }

      for (size_t i = 0; i < num_steps; ++i) {
        double s = geom_s_start + (static_cast<double>(i) * geom_length / num_steps);
        stations.push_back(s);
      }
    }
    stations.push_back(road_len);

    // Sort and remove duplicates from stations
    std::ranges::sort(stations);
    auto [first, last] =
        std::ranges::unique(stations, [](double a, double b) noexcept -> bool { return std::abs(a - b) < 1e-4; });
    stations.erase(first, last);

    // --- Part A: Tessellate Reference Line ---
    Polyline ref_line;
    ref_line.road_id = road_id;
    ref_line.is_reference_line = true;
    ref_line.marking_type = "solid";

    for (double s : stations) {
      cpm::RoadPose pose = {s, 0.0, 0.0, 0.0, 0.0, 0.0, road_id};
      cpm::InertialPose ip = model.RoadToInertial(pose, ctx);
      ref_line.vertices.push_back(Vertex{static_cast<float>(ip.x), static_cast<float>(ip.y), static_cast<float>(ip.z)});
    }
    polylines_.push_back(ref_line);

    // --- Part B: Tessellate Lane Sections ---
    for (size_t sec_idx = 0; sec_idx < road.lanes.sections.size(); ++sec_idx) {
      const auto& section = road.lanes.sections[sec_idx];
      double sec_s_start = section.s;
      double sec_s_end = (sec_idx + 1 < road.lanes.sections.size()) ? road.lanes.sections[sec_idx + 1].s : road_len;

      // Gather stations within this section
      std::vector<double> sec_stations;
      for (double s : stations) {
        if (s >= sec_s_start && s <= sec_s_end) {
          sec_stations.push_back(s);
        }
      }

      if (sec_stations.empty() || std::abs(sec_stations.front() - sec_s_start) > 1e-4) {
        sec_stations.insert(sec_stations.begin(), sec_s_start);
      }
      if (std::abs(sec_stations.back() - sec_s_end) > 1e-4) {
        sec_stations.push_back(sec_s_end);
      }

      // Gather all lanes (left and right) in this section
      std::vector<const ast::Lane*> section_lanes;
      int max_left_id = 0;
      int min_right_id = 0;

      for (const auto& lane : section.left) {
        section_lanes.push_back(&lane);
        max_left_id = std::max(max_left_id, lane.id);
      }
      for (const auto& lane : section.right) {
        section_lanes.push_back(&lane);
        min_right_id = std::min(min_right_id, lane.id);
      }

      // Map AST lanes to absolute CPM LaneIds by replicating construction loop logic
      auto GetLaneId = [&](cpm::RoadId r_id, size_t s_idx, int original_id) -> cpm::LaneId {
        size_t absolute_lane_idx = 0;
        bool found = false;
        for (size_t r_i = 0; r_i < map.roads.size(); ++r_i) {
          const auto& r = map.roads[r_i];
          for (size_t s_i = 0; s_i < r.lanes.sections.size(); ++s_i) {
            const auto& sec = r.lanes.sections[s_i];
            std::vector<int> sorted_ids;
            for (const auto& l : sec.right) {
              sorted_ids.push_back(l.id);
            }
            for (const auto& l : sec.center) {
              sorted_ids.push_back(l.id);
            }
            for (const auto& l : sec.left) {
              sorted_ids.push_back(l.id);
            }
            std::ranges::sort(sorted_ids);

            for (int id : sorted_ids) {
              if (static_cast<cpm::RoadId>(r_i) == r_id && s_i == s_idx && id == original_id) {
                found = true;
                break;
              }
              if (!found) {
                absolute_lane_idx++;
              }
            }
            if (found) {
              break;
            }
          }
          if (found) {
            break;
          }
        }
        return static_cast<cpm::LaneId>(absolute_lane_idx);
      };

      // Generate mesh surfaces and outer boundary polylines for each lane
      for (const auto* lane_ptr : section_lanes) {
        const auto& lane = *lane_ptr;
        auto lane_id = GetLaneId(road_id, sec_idx, lane.id);

        Mesh mesh;
        mesh.road_id = road_id;
        mesh.lane_id = lane_id;
        mesh.lane_type = lane.type;

        Polyline boundary;
        boundary.road_id = road_id;
        boundary.original_lane_id = lane.id;
        boundary.is_reference_line = false;
        boundary.marking_type = (lane.id > 0) ? (lane.id == max_left_id ? "solid" : "broken")
                                              : (lane.id == min_right_id ? "solid" : "broken");

        for (double s : sec_stations) {
          double w_target = model.LaneWidth(lane_id, s);

          // Boundaries in lane-local track coordinates
          double t_inner = 0.0;
          double t_outer = 0.0;
          if (lane.id > 0) {
            t_inner = -0.5 * w_target;
            t_outer = 0.5 * w_target;
          } else {
            t_inner = 0.5 * w_target;
            t_outer = -0.5 * w_target;
          }

          // Evaluate 3D Inertial coordinates for inner and outer boundaries
          cpm::LanePose lp_inner = {s, t_inner, 0.0, 0.0, 0.0, 0.0, road_id, lane_id};
          cpm::RoadPose rp_inner = model.LaneToRoad(lp_inner, ctx);
          cpm::InertialPose ip_inner = model.RoadToInertial(rp_inner, ctx);

          cpm::LanePose lp_outer = {s, t_outer, 0.0, 0.0, 0.0, 0.0, road_id, lane_id};
          cpm::RoadPose rp_outer = model.LaneToRoad(lp_outer, ctx);
          cpm::InertialPose ip_outer = model.RoadToInertial(rp_outer, ctx);

          Vertex v_inner = {static_cast<float>(ip_inner.x), static_cast<float>(ip_inner.y),
                            static_cast<float>(ip_inner.z)};
          Vertex v_outer = {static_cast<float>(ip_outer.x), static_cast<float>(ip_outer.y),
                            static_cast<float>(ip_outer.z)};

          mesh.vertices.push_back(v_inner);
          mesh.vertices.push_back(v_outer);

          boundary.vertices.push_back(v_outer);
        }

        // Build indices for the surface triangles (CCW winding)
        for (size_t j = 0; j < sec_stations.size() - 1; ++j) {
          auto idx_a = static_cast<uint32_t>(2 * j);
          auto idx_b = static_cast<uint32_t>(2 * j + 1);
          auto idx_c = static_cast<uint32_t>(2 * (j + 1));
          auto idx_d = static_cast<uint32_t>(2 * (j + 1) + 1);

          if (lane.id > 0) {
            mesh.indices.push_back(idx_a);
            mesh.indices.push_back(idx_d);
            mesh.indices.push_back(idx_b);

            mesh.indices.push_back(idx_a);
            mesh.indices.push_back(idx_c);
            mesh.indices.push_back(idx_d);
          } else {
            // Reverse winding order for right lanes to maintain CCW orientation
            mesh.indices.push_back(idx_a);
            mesh.indices.push_back(idx_b);
            mesh.indices.push_back(idx_d);

            mesh.indices.push_back(idx_a);
            mesh.indices.push_back(idx_d);
            mesh.indices.push_back(idx_c);
          }
        }

        meshes_.push_back(mesh);
        polylines_.push_back(boundary);
      }
    }
  }
}

}  // namespace strada::tess
