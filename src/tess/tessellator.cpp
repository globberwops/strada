// SPDX-License-Identifier: BSL-1.0

#include <algorithm>
#include <cmath>
#include <strada/cpm/compiled_physics_model.hpp>
#include <strada/cpm/query_context.hpp>
#include <strada/tess/tessellator.hpp>
#include <variant>

#include "triangulation.hpp"

namespace strada::tess {

Tessellator::Tessellator(const ast::AbstractSyntaxTree& map, double chord_error) {
  auto model = cpm::CompiledPhysicsModel::Build(map);
  cpm::QueryContext ctx;

  for (std::size_t road_idx = 0; road_idx < map.roads.size(); ++road_idx) {
    const auto& road = map.roads[road_idx];
    auto road_id = static_cast<cpm::RoadId>(road_idx);
    auto stations = ComputeSamplingStations(road, chord_error);

    TessellateReferenceLine(road_id, stations, model, ctx);
    TessellateLaneSections(road, road_id, stations, model, ctx, map);
  }

  TessellateJunctionBoundaries(map, model, ctx, chord_error);
}

auto Tessellator::ComputeSamplingStations(const ast::Road& road, double chord_error) -> std::vector<double> {
  std::vector<double> stations;
  const double kRoadLen = road.length;

  for (const auto& geom : road.plan_view) {
    const double kGeomSStart = geom.s;
    const double kGeomLength = geom.length;

    // Determine step count based on geometry type and chord error
    std::size_t num_steps = 10;  // Default fallback

    if (std::holds_alternative<ast::Line>(geom.shape)) {
      num_steps = 1;
    } else if (const auto* arc_ptr = std::get_if<ast::Arc>(&geom.shape)) {
      const double kCurvature = std::abs(arc_ptr->curvature);
      if (kCurvature > 1e-6) {
        const double kRadius = 1.0 / kCurvature;
        double ds = std::sqrt(8.0 * kRadius * chord_error);
        ds = std::clamp(ds, 0.2, 5.0);  // Clamp step size to reasonable bounds
        num_steps = static_cast<std::size_t>(std::ceil(kGeomLength / ds));
      }
    } else {
      // Fallback for spirals and polynomials
      const double kDs = std::clamp(chord_error * 5.0, 0.5, 2.0);
      num_steps = static_cast<std::size_t>(std::ceil(kGeomLength / kDs));
    }

    num_steps = std::max<std::size_t>(num_steps, 1);

    for (std::size_t i = 0; i < num_steps; ++i) {
      const double kS = kGeomSStart + (static_cast<double>(i) * kGeomLength / static_cast<double>(num_steps));
      stations.push_back(kS);
    }
  }
  stations.push_back(kRoadLen);

  // Sort and remove duplicates from stations
  std::ranges::sort(stations);
  auto [first, last] =
      std::ranges::unique(stations, [](double a, double b) noexcept -> bool { return std::abs(a - b) < 1e-4; });
  stations.erase(first, last);

  return stations;
}

void Tessellator::TessellateReferenceLine(cpm::RoadId road_id, const std::vector<double>& stations,
                                          const cpm::CompiledPhysicsModel& model, cpm::QueryContext& ctx) {
  Polyline ref_line;
  ref_line.road_id = road_id;
  ref_line.is_reference_line = true;
  ref_line.marking_type = "solid";

  for (const double kS : stations) {
    const cpm::RoadPose kPose = {
        .s = kS, .t = 0.0, .h = 0.0, .heading = 0.0, .pitch = 0.0, .roll = 0.0, .road = road_id};
    const cpm::InertialPose kIp = model.RoadToInertial(kPose, ctx);
    ref_line.vertices.push_back(
        Vertex{.x = static_cast<float>(kIp.x), .y = static_cast<float>(kIp.y), .z = static_cast<float>(kIp.z)});
  }
  polylines_.push_back(ref_line);
}

void Tessellator::TessellateLaneSections(const ast::Road& road, cpm::RoadId road_id,
                                         const std::vector<double>& stations, const cpm::CompiledPhysicsModel& model,
                                         cpm::QueryContext& ctx, const ast::AbstractSyntaxTree& map) {
  const double kRoadLen = road.length;

  for (std::size_t sec_idx = 0; sec_idx < road.lanes.sections.size(); ++sec_idx) {
    const auto& section = road.lanes.sections[sec_idx];
    const double kSecSStart = section.s;
    const double kSecSEnd = (sec_idx + 1 < road.lanes.sections.size()) ? road.lanes.sections[sec_idx + 1].s : kRoadLen;

    // Gather stations within this section
    std::vector<double> sec_stations;
    for (const double kS : stations) {
      if (kS >= kSecSStart && kS <= kSecSEnd) {
        sec_stations.push_back(kS);
      }
    }

    if (sec_stations.empty() || std::abs(sec_stations.front() - kSecSStart) > 1e-4) {
      sec_stations.insert(sec_stations.begin(), kSecSStart);
    }
    if (std::abs(sec_stations.back() - kSecSEnd) > 1e-4) {
      sec_stations.push_back(kSecSEnd);
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

    // Generate mesh surfaces and outer boundary polylines for each lane
    for (const auto* lane_ptr : section_lanes) {
      const auto& lane = *lane_ptr;
      auto lane_id = model.FindLaneId(road_id, sec_idx, lane.id).value();

      Mesh mesh;
      mesh.road_id = road_id;
      mesh.lane_id = lane_id;
      mesh.lane_type = std::string(ast::ToString(lane.type));

      Polyline boundary;
      boundary.road_id = road_id;
      boundary.original_lane_id = lane.id;
      boundary.is_reference_line = false;
      boundary.marking_type = (lane.id > 0) ? (lane.id == max_left_id ? "solid" : "broken")
                                            : (lane.id == min_right_id ? "solid" : "broken");

      for (const double kS : sec_stations) {
        const double kWTarget = model.LaneWidth(lane_id, kS);

        // Boundaries in lane-local track coordinates
        double t_inner = 0.0;
        double t_outer = 0.0;
        if (lane.id > 0) {
          t_inner = -0.5 * kWTarget;
          t_outer = 0.5 * kWTarget;
        } else {
          t_inner = 0.5 * kWTarget;
          t_outer = -0.5 * kWTarget;
        }

        // Evaluate 3D Inertial coordinates for inner and outer boundaries
        const cpm::LanePose kLpInner = {.s = kS,
                                        .t = t_inner,
                                        .h = 0.0,
                                        .heading = 0.0,
                                        .pitch = 0.0,
                                        .roll = 0.0,
                                        .road = road_id,
                                        .lane = lane_id};
        const cpm::RoadPose kRpInner = model.LaneToRoad(kLpInner, ctx);
        const cpm::InertialPose kIpInner = model.RoadToInertial(kRpInner, ctx);

        const cpm::LanePose kLpOuter = {.s = kS,
                                        .t = t_outer,
                                        .h = 0.0,
                                        .heading = 0.0,
                                        .pitch = 0.0,
                                        .roll = 0.0,
                                        .road = road_id,
                                        .lane = lane_id};
        const cpm::RoadPose kRpOuter = model.LaneToRoad(kLpOuter, ctx);
        const cpm::InertialPose kIpOuter = model.RoadToInertial(kRpOuter, ctx);

        const Vertex kVInner = {.x = static_cast<float>(kIpInner.x),
                                .y = static_cast<float>(kIpInner.y),
                                .z = static_cast<float>(kIpInner.z)};
        const Vertex kVOuter = {.x = static_cast<float>(kIpOuter.x),
                                .y = static_cast<float>(kIpOuter.y),
                                .z = static_cast<float>(kIpOuter.z)};

        mesh.vertices.push_back(kVInner);
        mesh.vertices.push_back(kVOuter);

        boundary.vertices.push_back(kVOuter);
      }

      // Build indices for the surface triangles (CCW winding)
      for (std::size_t j = 0; j < sec_stations.size() - 1; ++j) {
        auto idx_a = static_cast<std::uint32_t>(2 * j);
        auto idx_b = static_cast<std::uint32_t>((2 * j) + 1);
        auto idx_c = static_cast<std::uint32_t>(2 * (j + 1));
        auto idx_d = static_cast<std::uint32_t>((2 * (j + 1)) + 1);

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

void Tessellator::TessellateJunctionBoundaries(const ast::AbstractSyntaxTree& map,
                                               const cpm::CompiledPhysicsModel& model, cpm::QueryContext& ctx,
                                               double chord_error) {
  for (const auto& junction : map.junctions) {
    if (!junction.boundary.has_value()) {
      std::vector<Vertex> fallback_vertices;
      std::vector<std::uint32_t> fallback_indices;

      std::vector<cpm::RoadId> connecting_roads;
      for (std::size_t r_i = 0; r_i < map.roads.size(); ++r_i) {
        if (map.roads[r_i].junction == junction.id) {
          connecting_roads.push_back(static_cast<cpm::RoadId>(r_i));
        }
      }

      for (const auto& mesh : meshes_) {
        if (std::ranges::find(connecting_roads, mesh.road_id) != connecting_roads.end()) {
          auto vertex_offset = static_cast<std::uint32_t>(fallback_vertices.size());
          for (const auto& v : mesh.vertices) {
            fallback_vertices.push_back(v);
          }
          for (const std::uint32_t kIdx : mesh.indices) {
            fallback_indices.push_back(kIdx + vertex_offset);
          }
        }
      }

      if (!fallback_vertices.empty()) {
        JunctionBoundaryGeometry geom;
        geom.junction_id = junction.id;
        geom.vertices = fallback_vertices;
        geom.indices = fallback_indices;
        junction_boundaries_.push_back(geom);
      }
      continue;
    }

    std::vector<Vertex> loop_vertices;

    for (const auto& segment : junction.boundary->segments) {
      // Find road index
      std::size_t road_idx = 0;
      bool found_road = false;
      for (std::size_t i = 0; i < map.roads.size(); ++i) {
        if (map.roads[i].id == segment.road_id) {
          road_idx = i;
          found_road = true;
          break;
        }
      }

      if (!found_road) {
        continue;
      }

      const auto& road = map.roads[road_idx];
      auto road_id = static_cast<cpm::RoadId>(road_idx);

      if (segment.type == ast::JunctionSegmentType::kLane) {
        if (!segment.boundary_lane.has_value()) {
          continue;
        }
        const int kBl = *segment.boundary_lane;
        const double kStartS = segment.s_start;
        double end_s = segment.s_end;
        if (std::isinf(end_s)) {
          end_s = road.length;
        }

        const double kSegLen = std::abs(end_s - kStartS);
        const double kDs = std::clamp(chord_error * 5.0, 0.5, 2.0);
        auto num_steps = static_cast<std::size_t>(std::ceil(kSegLen / kDs));
        num_steps = std::max<std::size_t>(num_steps, 2);

        for (std::size_t k = 0; k <= num_steps; ++k) {
          const double kS = kStartS + (static_cast<double>(k) * (end_s - kStartS) / static_cast<double>(num_steps));

          // Find active lane section at s
          std::size_t s_idx = 0;
          for (std::size_t idx = 0; idx < road.lanes.sections.size(); ++idx) {
            if (kS >= road.lanes.sections[idx].s) {
              s_idx = idx;
            } else {
              break;
            }
          }

          auto lane_id = model.FindLaneId(road_id, s_idx, kBl).value();
          const double kW = model.LaneWidth(lane_id, kS);

          const cpm::LanePose kLp = {.s = kS,
                                     .t = (kBl > 0) ? (0.5 * kW) : (-0.5 * kW),
                                     .h = 0.0,
                                     .heading = 0.0,
                                     .pitch = 0.0,
                                     .roll = 0.0,
                                     .road = road_id,
                                     .lane = lane_id};
          const cpm::RoadPose kRp = model.LaneToRoad(kLp, ctx);
          const cpm::InertialPose kIp = model.RoadToInertial(kRp, ctx);

          loop_vertices.push_back(
              Vertex{.x = static_cast<float>(kIp.x), .y = static_cast<float>(kIp.y), .z = static_cast<float>(kIp.z)});
        }
      } else {
        double s = (segment.contact_point == ast::ContactPoint::kStart) ? 0.0 : road.length;

        // Find active lane section at s
        std::size_t s_idx = 0;
        for (std::size_t idx = 0; idx < road.lanes.sections.size(); ++idx) {
          if (s >= road.lanes.sections[idx].s) {
            s_idx = idx;
          } else {
            break;
          }
        }
        const auto& section = road.lanes.sections[s_idx];

        int max_left_id = 0;
        int min_right_id = 0;
        for (const auto& l : section.left) {
          max_left_id = std::max(max_left_id, l.id);
        }
        for (const auto& l : section.right) {
          min_right_id = std::min(min_right_id, l.id);
        }

        const int kLStart = segment.joint_lane_start.value_or(min_right_id);
        const int kLEnd = segment.joint_lane_end.value_or(max_left_id);

        auto get_outer_t = [&](int lane_id_val) -> double {
          if (lane_id_val == 0) {
            return 0.0;
          }
          auto l_id = model.FindLaneId(road_id, s_idx, lane_id_val).value();
          const double kW = model.LaneWidth(l_id, s);
          const cpm::LanePose kLp = {.s = s,
                                     .t = (lane_id_val > 0) ? (0.5 * kW) : (-0.5 * kW),
                                     .h = 0.0,
                                     .heading = 0.0,
                                     .pitch = 0.0,
                                     .roll = 0.0,
                                     .road = road_id,
                                     .lane = l_id};
          const cpm::RoadPose kRp = model.LaneToRoad(kLp, ctx);
          return kRp.t;
        };

        const double kTStart = get_outer_t(kLStart);
        const double kTEnd = get_outer_t(kLEnd);

        const double kTDiff = std::abs(kTEnd - kTStart);
        const double kDt = std::clamp(chord_error * 5.0, 0.5, 2.0);
        auto num_steps = static_cast<std::size_t>(std::ceil(kTDiff / kDt));
        num_steps = std::max<std::size_t>(num_steps, 2);

        for (std::size_t k = 0; k <= num_steps; ++k) {
          const double kT = kTStart + (static_cast<double>(k) * (kTEnd - kTStart) / static_cast<double>(num_steps));
          const cpm::RoadPose kRp = {
              .s = s, .t = kT, .h = 0.0, .heading = 0.0, .pitch = 0.0, .roll = 0.0, .road = road_id};
          const cpm::InertialPose kIp = model.RoadToInertial(kRp, ctx);
          loop_vertices.push_back(
              Vertex{.x = static_cast<float>(kIp.x), .y = static_cast<float>(kIp.y), .z = static_cast<float>(kIp.z)});
        }
      }
    }

    // Deduplicate adjacent identical vertices
    if (loop_vertices.size() > 1) {
      auto [uniq_first, uniq_last] =
          std::ranges::unique(loop_vertices, [](const Vertex& a, const Vertex& b) noexcept -> bool {
            return std::abs(a.x - b.x) < 1e-4F && std::abs(a.y - b.y) < 1e-4F && std::abs(a.z - b.z) < 1e-4F;
          });
      loop_vertices.erase(uniq_first, uniq_last);

      // If the last vertex is identical to the first, remove it to make the loop strictly open-ended
      if (loop_vertices.size() > 2) {
        const auto& first = loop_vertices.front();
        const auto& last = loop_vertices.back();
        if (std::abs(first.x - last.x) < 1e-4F && std::abs(first.y - last.y) < 1e-4F &&
            std::abs(first.z - last.z) < 1e-4F) {
          loop_vertices.pop_back();
        }
      }
    }

    if (loop_vertices.size() >= 3) {
      JunctionBoundaryGeometry geom;
      geom.junction_id = junction.id;
      geom.outline_vertices = loop_vertices;
      geom.outline_vertices.push_back(loop_vertices.front());

      geom.vertices = loop_vertices;
      geom.indices = TriangulatePolygon(loop_vertices);

      junction_boundaries_.push_back(geom);
    }
  }
}

}  // namespace strada::tess
