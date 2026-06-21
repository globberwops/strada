// SPDX-License-Identifier: BSL-1.0

#include <algorithm>
#include <cmath>
#include <strada/cpm/compiled_physics_model.hpp>
#include <strada/cpm/query_context.hpp>
#include <strada/tess/tessellator.hpp>
#include <variant>

namespace strada::tess {

namespace {

auto TriangulatePolygon(const std::vector<Vertex>& vertices) -> std::vector<uint32_t> {
  std::vector<uint32_t> indices;
  if (vertices.size() < 3) {
    return indices;
  }

  size_t n = vertices.size();
  std::vector<uint32_t> v(n);

  // Compute signed area to determine winding order
  double area = 0.0;
  for (size_t i = 0; i < n; ++i) {
    const auto& p1 = vertices[i];
    const auto& p2 = vertices[(i + 1) % n];
    area += (static_cast<double>(p1.x) * p2.y) - (static_cast<double>(p2.x) * p1.y);
  }

  // Winding order: if area is negative, we want CW, if positive CCW.
  if (area < 0.0) {
    for (size_t i = 0; i < n; ++i) {
      v[i] = static_cast<uint32_t>(n - 1 - i);
    }
  } else {
    for (size_t i = 0; i < n; ++i) {
      v[i] = static_cast<uint32_t>(i);
    }
  }

  // Helper functions inside TriangulatePolygon
  auto inside_triangle = [](float ax, float ay, float bx, float by, float cx, float cy, float px, float py) -> bool {
    float ax_px = ax - px;
    float ay_py = ay - py;
    float bx_px = bx - px;
    float by_py = by - py;
    float cx_px = cx - px;
    float cy_py = cy - py;

    float ccw_ab = ax_px * by_py - ay_py * bx_px;
    float ccw_bc = bx_px * cy_py - by_py * cx_px;
    float ccw_ca = cx_px * ay_py - cy_py * ax_px;

    return (ccw_ab >= 0.0f && ccw_bc >= 0.0f && ccw_ca >= 0.0f) || (ccw_ab <= 0.0f && ccw_bc <= 0.0f && ccw_ca <= 0.0f);
  };

  auto is_ear = [&](size_t u, size_t w, size_t cv, const std::vector<uint32_t>& v_indices) -> bool {
    const auto& a = vertices[v_indices[u]];
    const auto& b = vertices[v_indices[w]];
    const auto& c = vertices[v_indices[cv]];

    // Check if triangle is convex (CCW)
    float cross_product = (b.x - a.x) * (c.y - b.y) - (b.y - a.y) * (c.x - b.x);
    if (cross_product <= 0.0f) {
      return false;
    }

    // Check if any other vertex is inside the triangle
    for (size_t p = 0; p < v_indices.size(); ++p) {
      if (p == u || p == w || p == cv) {
        continue;
      }
      const auto& pt = vertices[v_indices[p]];
      if (inside_triangle(a.x, a.y, b.x, b.y, c.x, c.y, pt.x, pt.y)) {
        return false;
      }
    }
    return true;
  };

  size_t count = 2 * n;  // Prevent infinite loop on degenerate polygons
  size_t nv = n;
  while (nv > 2) {
    if (count == 0) {
      // Degenerate fallback to triangle fan to avoid hanging
      for (size_t i = 1; i < nv - 1; ++i) {
        indices.push_back(v[0]);
        indices.push_back(v[i]);
        indices.push_back(v[i + 1]);
      }
      return indices;
    }
    count--;

    for (size_t i = 0; i < nv; ++i) {
      size_t u = (i == 0) ? (nv - 1) : (i - 1);
      size_t w = i;
      size_t cv = (i + 1 == nv) ? 0 : (i + 1);

      if (is_ear(u, w, cv, v)) {
        indices.push_back(v[u]);
        indices.push_back(v[w]);
        indices.push_back(v[cv]);

        v.erase(v.begin() + static_cast<std::vector<uint32_t>::difference_type>(w));
        nv--;
        count = 2 * nv;
        break;
      }
    }
  }

  return indices;
}

}  // namespace

Tessellator::Tessellator(const ast::AbstractSyntaxTree& map, double chord_error) {
  // 1. Build a temporary CompiledPhysicsModel to evaluate geometries
  auto model = cpm::CompiledPhysicsModel::Build(map);
  cpm::QueryContext ctx;

  // Map AST lanes to absolute CPM LaneIds by replicating construction loop logic
  auto get_lane_id = [&](cpm::RoadId r_id, size_t s_idx, int original_id) -> cpm::LaneId {
    size_t absolute_lane_idx = 0;
    bool found = false;
    for (size_t r_i = 0; r_i < map.roads.size(); ++r_i) {
      const auto& r = map.roads[r_i];
      for (size_t s_i = 0; s_i < r.lanes.sections.size(); ++s_i) {
        const auto& sec = r.lanes.sections[s_i];
        std::vector<int> sorted_ids;
        sorted_ids.reserve(sec.right.size());
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
      } else if (const auto* arc_ptr = std::get_if<ast::Arc>(&geom.shape)) {
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

      num_steps = std::max<size_t>(num_steps, 1);

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
      cpm::RoadPose pose = {.s = s, .t = 0.0, .h = 0.0, .heading = 0.0, .pitch = 0.0, .roll = 0.0, .road = road_id};
      cpm::InertialPose ip = model.RoadToInertial(pose, ctx);
      ref_line.vertices.push_back(
          Vertex{.x = static_cast<float>(ip.x), .y = static_cast<float>(ip.y), .z = static_cast<float>(ip.z)});
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

      // Generate mesh surfaces and outer boundary polylines for each lane
      for (const auto* lane_ptr : section_lanes) {
        const auto& lane = *lane_ptr;
        auto lane_id = get_lane_id(road_id, sec_idx, lane.id);

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
          cpm::LanePose lp_inner = {.s = s,
                                    .t = t_inner,
                                    .h = 0.0,
                                    .heading = 0.0,
                                    .pitch = 0.0,
                                    .roll = 0.0,
                                    .road = road_id,
                                    .lane = lane_id};
          cpm::RoadPose rp_inner = model.LaneToRoad(lp_inner, ctx);
          cpm::InertialPose ip_inner = model.RoadToInertial(rp_inner, ctx);

          cpm::LanePose lp_outer = {.s = s,
                                    .t = t_outer,
                                    .h = 0.0,
                                    .heading = 0.0,
                                    .pitch = 0.0,
                                    .roll = 0.0,
                                    .road = road_id,
                                    .lane = lane_id};
          cpm::RoadPose rp_outer = model.LaneToRoad(lp_outer, ctx);
          cpm::InertialPose ip_outer = model.RoadToInertial(rp_outer, ctx);

          Vertex v_inner = {.x = static_cast<float>(ip_inner.x),
                            .y = static_cast<float>(ip_inner.y),
                            .z = static_cast<float>(ip_inner.z)};
          Vertex v_outer = {.x = static_cast<float>(ip_outer.x),
                            .y = static_cast<float>(ip_outer.y),
                            .z = static_cast<float>(ip_outer.z)};

          mesh.vertices.push_back(v_inner);
          mesh.vertices.push_back(v_outer);

          boundary.vertices.push_back(v_outer);
        }

        // Build indices for the surface triangles (CCW winding)
        for (size_t j = 0; j < sec_stations.size() - 1; ++j) {
          auto idx_a = static_cast<uint32_t>(2 * j);
          auto idx_b = static_cast<uint32_t>((2 * j) + 1);
          auto idx_c = static_cast<uint32_t>(2 * (j + 1));
          auto idx_d = static_cast<uint32_t>((2 * (j + 1)) + 1);

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

  // --- Part C: Tessellate Junction Boundaries ---
  for (const auto& junction : map.junctions) {
    if (!junction.boundary.has_value()) {
      continue;
    }

    std::vector<Vertex> loop_vertices;

    for (const auto& segment : junction.boundary->segments) {
      // Find road index
      size_t road_idx = 0;
      bool found_road = false;
      for (size_t i = 0; i < map.roads.size(); ++i) {
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
        int bl = *segment.boundary_lane;
        double start_s = segment.s_start;
        double end_s = segment.s_end;
        if (std::isinf(end_s)) {
          end_s = road.length;
        }

        double seg_len = std::abs(end_s - start_s);
        double ds = std::clamp(chord_error * 5.0, 0.5, 2.0);
        size_t num_steps = static_cast<size_t>(std::ceil(seg_len / ds));
        num_steps = std::max<size_t>(num_steps, 2);

        for (size_t k = 0; k <= num_steps; ++k) {
          double s = start_s + (static_cast<double>(k) * (end_s - start_s) / num_steps);

          // Find active lane section at s
          size_t s_idx = 0;
          for (size_t idx = 0; idx < road.lanes.sections.size(); ++idx) {
            if (s >= road.lanes.sections[idx].s) {
              s_idx = idx;
            } else {
              break;
            }
          }

          auto lane_id = get_lane_id(road_id, s_idx, bl);
          double w = model.LaneWidth(lane_id, s);

          cpm::LanePose lp = {.s = s,
                              .t = (bl > 0) ? (0.5 * w) : (-0.5 * w),
                              .h = 0.0,
                              .heading = 0.0,
                              .pitch = 0.0,
                              .roll = 0.0,
                              .road = road_id,
                              .lane = lane_id};
          cpm::RoadPose rp = model.LaneToRoad(lp, ctx);
          cpm::InertialPose ip = model.RoadToInertial(rp, ctx);

          loop_vertices.push_back(
              Vertex{.x = static_cast<float>(ip.x), .y = static_cast<float>(ip.y), .z = static_cast<float>(ip.z)});
        }
      } else {
        // Slice 2: Joint boundaries will be implemented in the next ticket.
      }
    }

    // Deduplicate adjacent identical vertices
    if (loop_vertices.size() > 1) {
      auto it = std::unique(loop_vertices.begin(), loop_vertices.end(), [](const Vertex& a, const Vertex& b) noexcept {
        return std::abs(a.x - b.x) < 1e-4f && std::abs(a.y - b.y) < 1e-4f && std::abs(a.z - b.z) < 1e-4f;
      });
      loop_vertices.erase(it, loop_vertices.end());

      // If the last vertex is identical to the first, remove it to make the loop strictly open-ended
      if (loop_vertices.size() > 2) {
        const auto& first = loop_vertices.front();
        const auto& last = loop_vertices.back();
        if (std::abs(first.x - last.x) < 1e-4f && std::abs(first.y - last.y) < 1e-4f &&
            std::abs(first.z - last.z) < 1e-4f) {
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
