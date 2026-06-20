#include <algorithm>
#include <cmath>
#include <limits>
#include <span>
#include <strada/cpm/aligned_allocator.hpp>
#include <strada/cpm/compiled_physics_model.hpp>
#include <strada/cpm/geometry_math.hpp>

#include "rotation.hpp"

namespace strada::cpm {

namespace {

constexpr double kCurvatureThreshold = 1e-12;
constexpr double kPolyCoeff2 = 2.0;
constexpr double kPolyCoeff3 = 3.0;

auto EvaluatePolynomial(const PolynomialsSoA& poly, uint32_t first_idx, uint32_t count, double s_coord) noexcept
    -> double {
  if (count == 0) {
    return 0.0;
  }
  uint32_t active_idx = first_idx;
  for (uint32_t i = 0; i < count; ++i) {
    uint32_t idx = first_idx + i;
    if (s_coord >= poly.s_start[idx]) {
      active_idx = idx;
    } else {
      break;
    }
  }
  double ds_val = s_coord - poly.s_start[active_idx];
  return poly.a[active_idx] + (poly.b[active_idx] * ds_val) + (poly.c[active_idx] * ds_val * ds_val) +
         (poly.d[active_idx] * ds_val * ds_val * ds_val);
}

void CompileCoefficients(const std::vector<ast::Coefficient>& coeffs, PolynomialsSoA& dest, uint32_t& first_idx,
                         uint32_t& count) {
  first_idx = static_cast<uint32_t>(dest.s_start.size());
  count = static_cast<uint32_t>(coeffs.size());
  for (const auto& coeff : coeffs) {
    dest.s_start.push_back(coeff.s);
    dest.a.push_back(coeff.a);
    dest.b.push_back(coeff.b);
    dest.c.push_back(coeff.c);
    dest.d.push_back(coeff.d);
  }
}

auto EvaluateStripOwnHeight(const PolynomialsSoA& poly, const StripsSoA& strips, uint32_t strip_idx, double s_coord,
                            double dt_val) noexcept -> double {
  double coeff0 = EvaluatePolynomial(poly, strips.c0_first_idx[strip_idx], strips.c0_count[strip_idx], s_coord);
  double coeff1 = EvaluatePolynomial(poly, strips.c1_first_idx[strip_idx], strips.c1_count[strip_idx], s_coord);
  double coeff2 = EvaluatePolynomial(poly, strips.c2_first_idx[strip_idx], strips.c2_count[strip_idx], s_coord);
  double coeff3 = EvaluatePolynomial(poly, strips.c3_first_idx[strip_idx], strips.c3_count[strip_idx], s_coord);
  return coeff0 + (coeff1 * dt_val) + (coeff2 * dt_val * dt_val) + (coeff3 * dt_val * dt_val * dt_val);
}

auto EvaluateStripHeight(const PolynomialsSoA& poly, const StripsSoA& strips, uint32_t strip_idx,
                         uint32_t first_strip_idx, uint32_t strip_count, double s_coord, double dt_val) noexcept
    -> double {
  double h_accum = EvaluateStripOwnHeight(poly, strips, strip_idx, s_coord, dt_val);
  uint32_t curr_strip_idx = strip_idx;

  while (strips.is_relative[curr_strip_idx] != 0U) {
    int32_t id_val = strips.strip_id[curr_strip_idx];
    int32_t inner_id = (id_val > 0) ? (id_val - 1) : (id_val + 1);

    bool found = false;
    for (uint32_t j = 0; j < strip_count; ++j) {
      uint32_t inner_idx = first_strip_idx + j;
      if (strips.strip_id[inner_idx] == inner_id) {
        double inner_w =
            EvaluatePolynomial(poly, strips.width_first_idx[inner_idx], strips.width_count[inner_idx], s_coord);
        double inner_dt = (inner_id > 0) ? inner_w : -inner_w;
        h_accum += EvaluateStripOwnHeight(poly, strips, inner_idx, s_coord, inner_dt);
        curr_strip_idx = inner_idx;
        found = true;
        break;
      }
    }
    if (!found) {
      break;
    }
  }
  return h_accum;
}

void EvaluateNaturalOrientationAndElev(const PolynomialsSoA& polynomials,
                                       std::span<const uint32_t> road_elevation_first_idx,
                                       std::span<const uint32_t> road_elevation_count,
                                       std::span<const uint32_t> road_superelevation_first_idx,
                                       std::span<const uint32_t> road_superelevation_count,
                                       const RoadCrossSectionSurfaceSoA& road_css, uint32_t road_idx, double s_coord,
                                       double& elev, double& natural_pitch, double& natural_roll) noexcept {
  elev = EvaluatePolynomial(polynomials, road_elevation_first_idx[road_idx], road_elevation_count[road_idx], s_coord);

  double d_elev = 0.0;
  uint32_t elev_first_idx = road_elevation_first_idx[road_idx];
  uint32_t elev_count = road_elevation_count[road_idx];
  if (elev_count > 0) {
    uint32_t active_idx = elev_first_idx;
    for (uint32_t i = 0; i < elev_count; ++i) {
      uint32_t idx = elev_first_idx + i;
      if (s_coord >= polynomials.s_start[idx]) {
        active_idx = idx;
      } else {
        break;
      }
    }
    double ds_poly = s_coord - polynomials.s_start[active_idx];
    d_elev = polynomials.b[active_idx] + (kPolyCoeff2 * polynomials.c[active_idx] * ds_poly) +
             (kPolyCoeff3 * polynomials.d[active_idx] * ds_poly * ds_poly);
  }
  natural_pitch = std::atan(d_elev);

  natural_roll = 0.0;
  if (!road_css.strip_count.empty() && road_css.strip_count[road_idx] == 0) {
    natural_roll = EvaluatePolynomial(polynomials, road_superelevation_first_idx[road_idx],
                                      road_superelevation_count[road_idx], s_coord);
  }
}

void EvaluateCrossSectionSurfaceOffset(const PolynomialsSoA& polynomials, const StripsSoA& strips,
                                       const RoadCrossSectionSurfaceSoA& road_css, uint32_t road_idx, double s_coord,
                                       double t_coord, double& h_surf) noexcept {
  h_surf = 0.0;
  uint32_t css_strip_count = road_css.strip_count.empty() ? 0 : road_css.strip_count[road_idx];
  if (css_strip_count > 0) {
    double t_offset = EvaluatePolynomial(polynomials, road_css.t_offset_first_idx[road_idx],
                                         road_css.t_offset_count[road_idx], s_coord);

    double t_surf = t_coord - t_offset;
    bool is_left = (t_surf >= 0.0);
    double t_target = is_left ? t_surf : std::abs(t_surf);

    uint32_t first_strip_idx = road_css.first_strip_idx[road_idx];
    double t_accum = 0.0;

    for (uint32_t i = 0; i < css_strip_count; ++i) {
      uint32_t strip_idx = first_strip_idx + i;
      int32_t id_val = strips.strip_id[strip_idx];
      bool strip_is_left = (id_val > 0);

      if (strip_is_left == is_left) {
        double width_val = std::numeric_limits<double>::infinity();
        uint32_t w_count = strips.width_count[strip_idx];
        if (w_count > 0) {
          width_val = EvaluatePolynomial(polynomials, strips.width_first_idx[strip_idx], w_count, s_coord);
        }

        if (t_target >= t_accum && t_target < t_accum + width_val) {
          double t_strip = t_target - t_accum;
          double dt_val = strip_is_left ? t_strip : -t_strip;

          h_surf =
              EvaluateStripHeight(polynomials, strips, strip_idx, first_strip_idx, css_strip_count, s_coord, dt_val);
          break;
        }
        t_accum += width_val;
      }
    }
  }
}

auto EvaluateAstLaneWidth(const ast::Lane& lane, double s_local_to_section) noexcept -> double {
  if (lane.widths.empty()) {
    return 0.0;
  }
  const ast::LaneWidth* active = lane.widths.data();
  for (const auto& width_poly : lane.widths) {
    if (s_local_to_section >= width_poly.s_offset) {
      active = &width_poly;
    } else {
      break;
    }
  }
  double ds = s_local_to_section - active->s_offset;
  return active->a + (active->b * ds) + (active->c * ds * ds) + (active->d * ds * ds * ds);
}

inline auto DistancePointToAabb(double px, double py, double min_x, double min_y, double max_x, double max_y) noexcept
    -> double {
  double dx = std::max({0.0, min_x - px, px - max_x});
  double dy = std::max({0.0, min_y - py, py - max_y});
  return std::sqrt((dx * dx) + (dy * dy));
}

struct ShapeGroup {
  double s{};
  uint32_t first_idx{};
  uint32_t count{};
};

void FindShapeGroups(const ShapesSoA& shapes, uint32_t first_idx, uint32_t count, double s_coord,
                     std::optional<ShapeGroup>& g1, std::optional<ShapeGroup>& g2) noexcept {
  g1 = std::nullopt;
  g2 = std::nullopt;
  if (count == 0) {
    return;
  }

  double max_s_le = -std::numeric_limits<double>::max();
  double min_s_ge = std::numeric_limits<double>::max();
  bool found_le = false;
  bool found_ge = false;

  for (uint32_t i = 0; i < count; ++i) {
    double s_val = shapes.s[first_idx + i];
    if (s_val <= s_coord) {
      if (!found_le || s_val > max_s_le) {
        max_s_le = s_val;
        found_le = true;
      }
    }
    if (s_val >= s_coord) {
      if (!found_ge || s_val < min_s_ge) {
        min_s_ge = s_val;
        found_ge = true;
      }
    }
  }

  if (found_le) {
    ShapeGroup group;
    group.s = max_s_le;
    group.first_idx = 0;
    group.count = 0;
    bool in_group = false;
    for (uint32_t i = 0; i < count; ++i) {
      uint32_t idx = first_idx + i;
      if (shapes.s[idx] == max_s_le) {
        if (!in_group) {
          group.first_idx = idx;
          in_group = true;
        }
        group.count++;
      }
    }
    g1 = group;
  }

  if (found_ge) {
    ShapeGroup group;
    group.s = min_s_ge;
    group.first_idx = 0;
    group.count = 0;
    bool in_group = false;
    for (uint32_t i = 0; i < count; ++i) {
      uint32_t idx = first_idx + i;
      if (shapes.s[idx] == min_s_ge) {
        if (!in_group) {
          group.first_idx = idx;
          in_group = true;
        }
        group.count++;
      }
    }
    g2 = group;
  }
}

auto EvaluateGroupHeight(const ShapesSoA& shapes, const ShapeGroup& group, double t_coord) noexcept -> double {
  if (group.count == 0) {
    return 0.0;
  }
  uint32_t active_idx = group.first_idx;
  for (uint32_t i = 0; i < group.count; ++i) {
    uint32_t idx = group.first_idx + i;
    if (t_coord >= shapes.t[idx]) {
      active_idx = idx;
    } else {
      break;
    }
  }
  double dt = t_coord - shapes.t[active_idx];
  return shapes.a[active_idx] + (dt * (shapes.b[active_idx] + dt * (shapes.c[active_idx] + dt * shapes.d[active_idx])));
}

auto EvaluateGroupTGradient(const ShapesSoA& shapes, const ShapeGroup& group, double t_coord) noexcept -> double {
  if (group.count == 0) {
    return 0.0;
  }
  uint32_t active_idx = group.first_idx;
  for (uint32_t i = 0; i < group.count; ++i) {
    uint32_t idx = group.first_idx + i;
    if (t_coord >= shapes.t[idx]) {
      active_idx = idx;
    } else {
      break;
    }
  }
  double dt = t_coord - shapes.t[active_idx];
  return shapes.b[active_idx] + (dt * (2.0 * shapes.c[active_idx] + dt * 3.0 * shapes.d[active_idx]));
}

auto EvaluateShapeHeight(const ShapesSoA& shapes, uint32_t first_idx, uint32_t count, double s_coord,
                         double t_coord) noexcept -> double {
  std::optional<ShapeGroup> g1;
  std::optional<ShapeGroup> g2;
  FindShapeGroups(shapes, first_idx, count, s_coord, g1, g2);

  if (!g1.has_value() && !g2.has_value()) {
    return 0.0;
  }
  if (g1.has_value() && !g2.has_value()) {
    return EvaluateGroupHeight(shapes, *g1, t_coord);
  }
  if (!g1.has_value() && g2.has_value()) {
    return EvaluateGroupHeight(shapes, *g2, t_coord);
  }

  double h1 = EvaluateGroupHeight(shapes, *g1, t_coord);
  if (g1->s == g2->s) {
    return h1;
  }
  double h2 = EvaluateGroupHeight(shapes, *g2, t_coord);
  double f = (s_coord - g1->s) / (g2->s - g1->s);
  return ((1.0 - f) * h1) + (f * h2);
}

auto EvaluateShapeTGradient(const ShapesSoA& shapes, uint32_t first_idx, uint32_t count, double s_coord,
                            double t_coord) noexcept -> double {
  std::optional<ShapeGroup> g1;
  std::optional<ShapeGroup> g2;
  FindShapeGroups(shapes, first_idx, count, s_coord, g1, g2);

  if (!g1.has_value() && !g2.has_value()) {
    return 0.0;
  }
  if (g1.has_value() && !g2.has_value()) {
    return EvaluateGroupTGradient(shapes, *g1, t_coord);
  }
  if (!g1.has_value() && g2.has_value()) {
    return EvaluateGroupTGradient(shapes, *g2, t_coord);
  }

  double g1_val = EvaluateGroupTGradient(shapes, *g1, t_coord);
  if (g1->s == g2->s) {
    return g1_val;
  }
  double g2_val = EvaluateGroupTGradient(shapes, *g2, t_coord);
  double f = (s_coord - g1->s) / (g2->s - g1->s);
  return ((1.0 - f) * g1_val) + (f * g2_val);
}

}  // namespace

[[gnu::hot]] auto CompiledPhysicsModel::RoadToInertial(RoadPose pose, QueryContext& ctx) const noexcept
    -> InertialPose {
  auto road_idx = static_cast<uint32_t>(pose.road);
  auto [first_seg, seg_count] = ref_line_.GetRoadSegments(pose.road);
  if (seg_count == 0) {
    return InertialPose{};
  }

  // 1. Find segment index
  uint32_t seg_idx = ref_line_.FindSegmentIndex(pose.road, pose.s, ctx);

  // 2. Evaluate reference line
  auto pt = ref_line_.Evaluate(seg_idx, pose.s);

  // 3. Evaluate natural pitch, roll, and elevation
  double elev = 0.0;
  double natural_pitch = 0.0;
  double natural_roll = 0.0;
  EvaluateNaturalOrientationAndElev(polynomials_, elevation_.road_elevation_first_idx, elevation_.road_elevation_count,
                                    elevation_.road_superelevation_first_idx, elevation_.road_superelevation_count,
                                    road_css_, road_idx, pose.s, elev, natural_pitch, natural_roll);

  // 4. Cross section surface height offset
  double h_surf = 0.0;
  EvaluateCrossSectionSurfaceOffset(polynomials_, strips_, road_css_, road_idx, pose.s, pose.t, h_surf);

  double h_shape = EvaluateShapeHeight(shapes_, shapes_.road_shape_first_idx[road_idx],
                                       shapes_.road_shape_count[road_idx], pose.s, pose.t);
  double shape_grad = EvaluateShapeTGradient(shapes_, shapes_.road_shape_first_idx[road_idx],
                                             shapes_.road_shape_count[road_idx], pose.s, pose.t);

  // 5. Position composition
  double roll_total = natural_roll + std::atan(shape_grad);
  auto r_road = Rotation::FromEuler(pt.heading, natural_pitch, roll_total);

  double local_t = pose.t;
  double local_h = pose.h + h_surf + h_shape;

  auto offset = r_road.Transform(0.0, local_t, local_h);

  InertialPose inertial_pose;
  inertial_pose.x = pt.x + offset[0];
  inertial_pose.y = pt.y + offset[1];
  inertial_pose.z = elev + offset[2];

  // Composed orientation composition
  auto r_offset = Rotation::FromEuler(pose.heading, pose.pitch, pose.roll);
  auto r_inertial = r_road.Compose(r_offset);

  auto euler_angles = r_inertial.ToEuler();
  inertial_pose.heading = euler_angles.heading;
  inertial_pose.pitch = euler_angles.pitch;
  inertial_pose.roll = euler_angles.roll;

  return inertial_pose;
}

auto CompiledPhysicsModel::LaneToInertial(LanePose pose, QueryContext& ctx) const noexcept -> InertialPose {
  RoadPose road_pose = LaneToRoad(pose, ctx);
  return RoadToInertial(road_pose, ctx);
}

auto CompiledPhysicsModel::InertialToRoad(InertialPose pose, QueryContext& ctx) const noexcept
    -> std::optional<RoadPose> {
  auto snap_to_road = [&](uint32_t road_idx) noexcept -> std::optional<RoadPose> {
    auto [first_seg, seg_count] = ref_line_.GetRoadSegments(static_cast<RoadId>(road_idx));
    if (seg_count == 0) {
      return std::nullopt;
    }

    double min_dist_sq = std::numeric_limits<double>::max();
    double best_s = 0.0;
    double best_t = 0.0;
    double best_rhdg = 0.0;

    for (uint32_t i = 0; i < seg_count; ++i) {
      uint32_t seg_idx = first_seg + i;
      double global_s = ref_line_.Project(seg_idx, pose.x, pose.y);
      auto pt = ref_line_.Evaluate(seg_idx, global_s);

      double dx = pose.x - pt.x;
      double dy = pose.y - pt.y;
      double dist_sq = (dx * dx) + (dy * dy);
      if (dist_sq < min_dist_sq) {
        min_dist_sq = dist_sq;
        best_s = global_s;
        best_t = (-dx * std::sin(pt.heading)) + (dy * std::cos(pt.heading));
        best_rhdg = pt.heading;
      }
    }

    double elev = 0.0;
    double natural_pitch = 0.0;
    double natural_roll = 0.0;
    EvaluateNaturalOrientationAndElev(polynomials_, elevation_.road_elevation_first_idx,
                                      elevation_.road_elevation_count, elevation_.road_superelevation_first_idx,
                                      elevation_.road_superelevation_count, road_css_, road_idx, best_s, elev,
                                      natural_pitch, natural_roll);

    uint32_t best_seg_idx = ref_line_.FindSegmentIndex(static_cast<RoadId>(road_idx), best_s, ctx);
    auto pt = ref_line_.Evaluate(best_seg_idx, best_s);

    double dx = pose.x - pt.x;
    double dy = pose.y - pt.y;
    double dz = pose.z - elev;

    // Base roll calculation
    auto r_road_base = Rotation::FromEuler(best_rhdg, natural_pitch, natural_roll);
    double road_t_base = r_road_base.InverseTransform(dx, dy, dz)[1];

    // Shape evaluation and roll correction
    double shape_grad = EvaluateShapeTGradient(shapes_, shapes_.road_shape_first_idx[road_idx],
                                               shapes_.road_shape_count[road_idx], best_s, road_t_base);
    double roll_total = natural_roll + std::atan(shape_grad);

    auto r_road = Rotation::FromEuler(best_rhdg, natural_pitch, roll_total);
    double road_t = r_road.InverseTransform(dx, dy, dz)[1];

    double t_left = 0.0;
    double t_right = 0.0;
    GetRoadWidthLimits(road_idx, best_s, t_left, t_right);
    constexpr double kSnappingTolerance = 5.0;

    if (road_t >= t_right - kSnappingTolerance && road_t <= t_left + kSnappingTolerance) {
      RoadPose road_pose;
      road_pose.road = static_cast<RoadId>(road_idx);
      road_pose.s = best_s;
      road_pose.t = road_t;

      double h_surf = 0.0;
      EvaluateCrossSectionSurfaceOffset(polynomials_, strips_, road_css_, road_idx, best_s, road_t, h_surf);

      double h_shape = EvaluateShapeHeight(shapes_, shapes_.road_shape_first_idx[road_idx],
                                           shapes_.road_shape_count[road_idx], best_s, road_t);

      double local_h = r_road.InverseTransform(dx, dy, dz)[2];
      road_pose.h = local_h - h_surf - h_shape;

      auto r_inertial = Rotation::FromEuler(pose.heading, pose.pitch, pose.roll);
      auto r_offset = r_road.Inverse().Compose(r_inertial);
      auto offset_angles = r_offset.ToEuler();
      road_pose.heading = offset_angles.heading;
      road_pose.pitch = offset_angles.pitch;
      road_pose.roll = offset_angles.roll;

      return road_pose;
    }
    return std::nullopt;
  };

  // 1. Check temporal coherence fast path
  if (ctx.last_road.has_value()) {
    auto road_idx = static_cast<uint32_t>(*ctx.last_road);
    auto fast_pose = snap_to_road(road_idx);
    if (fast_pose.has_value()) {
      ref_line_.FindSegmentIndex(*ctx.last_road, fast_pose->s, ctx);
      return fast_pose;
    }
  }

  // 2. Traversal stack-based bounding volume hierarchy search
  std::optional<RoadPose> best_overall_pose;

  bounding_volume_hierarchy_.Query(
      pose.x, pose.y,
      [&](const BoundingVolumeHierarchy::PrimitiveInfo& prim, double current_min_dist) -> std::optional<double> {
        auto candidate = snap_to_road(prim.road_idx);
        if (candidate.has_value()) {
          double abs_t = std::abs(candidate->t);
          if (abs_t < current_min_dist) {
            best_overall_pose = candidate;
            return abs_t;
          }
        }
        return std::nullopt;
      });

  if (best_overall_pose.has_value()) {
    ref_line_.FindSegmentIndex(best_overall_pose->road, best_overall_pose->s, ctx);
  }

  return best_overall_pose;
}

auto CompiledPhysicsModel::InertialToLane(InertialPose pose, QueryContext& ctx) const noexcept
    -> std::optional<LanePose> {
  auto road_pose_opt = InertialToRoad(pose, ctx);
  if (!road_pose_opt.has_value()) {
    return std::nullopt;
  }
  return RoadToLane(*road_pose_opt, ctx);
}

auto CompiledPhysicsModel::RoadToLane(RoadPose pose, QueryContext& ctx) const noexcept -> std::optional<LanePose> {
  auto road_idx = static_cast<uint32_t>(pose.road);
  if (road_idx >= road_string_ids_.size()) {
    return std::nullopt;
  }

  auto road_sec_first = lane_sections_.road_section_first_idx[road_idx];
  auto road_sec_count = lane_sections_.road_section_count[road_idx];
  if (road_sec_count == 0) {
    return std::nullopt;
  }

  // Find the active lane section at pose.s
  auto sec_idx = road_sec_first;
  for (uint32_t i = 0; i < road_sec_count; ++i) {
    auto cur_sec = road_sec_first + i;
    if (pose.s >= lane_sections_.section_s[cur_sec]) {
      sec_idx = cur_sec;
    } else {
      break;
    }
  }

  auto first_lane_in_sec = lane_sections_.section_first_lane_idx[sec_idx];
  auto lane_cnt_in_sec = lane_sections_.section_lane_count[sec_idx];

  // Calculate road-level lane offset
  double lane_offset_val = 0.0;
  auto lo_first = lane_offsets_.road_lane_offset_first_idx[road_idx];
  auto lo_count = lane_offsets_.road_lane_offset_count[road_idx];
  if (lo_count > 0) {
    auto active_lo = lo_first;
    for (uint32_t i = 0; i < lo_count; ++i) {
      auto cur_lo = lo_first + i;
      if (pose.s >= lane_offsets_.lane_offset_s_start[cur_lo]) {
        active_lo = cur_lo;
      } else {
        break;
      }
    }
    double ds_lo = pose.s - lane_offsets_.lane_offset_s_start[active_lo];
    lane_offset_val =
        lane_offsets_.lane_offset_a[active_lo] +
        (ds_lo * (lane_offsets_.lane_offset_b[active_lo] +
                  ds_lo * (lane_offsets_.lane_offset_c[active_lo] + ds_lo * lane_offsets_.lane_offset_d[active_lo])));
  }

  double t_relative = pose.t - lane_offset_val;

  uint32_t matched_lane_idx = 0;
  bool found = false;
  double t_center = 0.0;
  double w_target = 0.0;
  int target_id = 0;

  if (t_relative > 0.0) {
    // Left lanes: IDs > 0, sorted ascending (e.g. 1, 2, 3...)
    double t_inner = 0.0;
    for (uint32_t i = 0; i < lane_cnt_in_sec; ++i) {
      uint32_t lane_idx = first_lane_in_sec + i;
      int lane_id = lanes_.lane_original_id[lane_idx];
      if (lane_id <= 0) {
        continue;
      }
      double w = LaneWidth(static_cast<LaneId>(lane_idx), pose.s);
      double t_outer = t_inner + w;
      if (w > 0.0 && t_relative >= t_inner && t_relative <= t_outer) {
        matched_lane_idx = lane_idx;
        t_center = t_inner + (0.5 * w);
        w_target = w;
        target_id = lane_id;
        found = true;
        break;
      }
      t_inner = t_outer;
    }
  } else if (t_relative < 0.0) {
    // Right lanes: IDs < 0, sorted ascending (e.g. -3, -2, -1)
    // Walk them in reverse order (from -1 down to -3) to go from inside to outside.
    double t_inner = 0.0;
    for (int i = static_cast<int>(lane_cnt_in_sec) - 1; i >= 0; --i) {
      uint32_t lane_idx = first_lane_in_sec + static_cast<uint32_t>(i);
      int lane_id = lanes_.lane_original_id[lane_idx];
      if (lane_id >= 0) {
        continue;
      }
      double w = LaneWidth(static_cast<LaneId>(lane_idx), pose.s);
      double t_outer = t_inner - w;
      if (w > 0.0 && t_relative <= t_inner && t_relative >= t_outer) {
        matched_lane_idx = lane_idx;
        t_center = t_inner - (0.5 * w);
        w_target = w;
        target_id = lane_id;
        found = true;
        break;
      }
      t_inner = t_outer;
    }
  }

  if (!found) {
    return std::nullopt;
  }

  // Evaluate lane height offset
  double h_inner = 0.0;
  double h_outer = 0.0;
  uint32_t h_first = lanes_.lane_first_height_idx[matched_lane_idx];
  uint32_t h_count = lanes_.lane_height_count[matched_lane_idx];
  if (h_count > 0) {
    uint32_t active_h = h_first;
    for (uint32_t i = 0; i < h_count; ++i) {
      uint32_t cur_h = h_first + i;
      if (pose.s >= lane_heights_.lane_height_s_start[cur_h]) {
        active_h = cur_h;
      } else {
        break;
      }
    }
    h_inner = lane_heights_.lane_height_inner[active_h];
    h_outer = lane_heights_.lane_height_outer[active_h];
  }

  double t_lane = t_relative - t_center;
  double f = 0.0;
  if (w_target > 0.0) {
    if (target_id > 0) {
      f = 0.5 + (t_lane / w_target);
    } else if (target_id < 0) {
      f = 0.5 - (t_lane / w_target);
    }
  }
  f = std::clamp(f, 0.0, 1.0);
  double h_offset = h_inner + (f * (h_outer - h_inner));

  LanePose lane_pose;
  lane_pose.s = pose.s;
  lane_pose.t = t_lane;
  lane_pose.h = pose.h - h_offset;
  lane_pose.heading = pose.heading;
  lane_pose.pitch = pose.pitch;
  lane_pose.roll = pose.roll;
  lane_pose.road = pose.road;
  lane_pose.lane = static_cast<LaneId>(matched_lane_idx);

  // Update query context road cache
  ctx.last_road = pose.road;

  return lane_pose;
}

void CompiledPhysicsModel::GetRoadWidthLimits(uint32_t road_idx, double s_coord, double& t_left,
                                              double& t_right) const noexcept {
  t_left = 0.0;
  t_right = 0.0;

  auto road_sec_first = lane_sections_.road_section_first_idx[road_idx];
  auto road_sec_count = lane_sections_.road_section_count[road_idx];
  if (road_sec_count == 0) {
    return;
  }

  auto sec_idx = road_sec_first;
  for (uint32_t i = 0; i < road_sec_count; ++i) {
    auto cur_sec = road_sec_first + i;
    if (s_coord >= lane_sections_.section_s[cur_sec]) {
      sec_idx = cur_sec;
    } else {
      break;
    }
  }

  auto first_lane_in_sec = lane_sections_.section_first_lane_idx[sec_idx];
  auto lane_cnt_in_sec = lane_sections_.section_lane_count[sec_idx];

  for (uint32_t i = 0; i < lane_cnt_in_sec; ++i) {
    auto lane_idx = first_lane_in_sec + i;
    auto lane_id = lanes_.lane_original_id[lane_idx];
    double w = LaneWidth(static_cast<LaneId>(lane_idx), s_coord);
    if (lane_id > 0) {
      t_left += w;
    } else if (lane_id < 0) {
      t_right -= w;
    }
  }

  double lane_offset_val = 0.0;
  auto lo_first = lane_offsets_.road_lane_offset_first_idx[road_idx];
  auto lo_count = lane_offsets_.road_lane_offset_count[road_idx];
  if (lo_count > 0) {
    auto active_lo = lo_first;
    for (uint32_t i = 0; i < lo_count; ++i) {
      auto cur_lo = lo_first + i;
      if (s_coord >= lane_offsets_.lane_offset_s_start[cur_lo]) {
        active_lo = cur_lo;
      } else {
        break;
      }
    }
    double ds_lo = s_coord - lane_offsets_.lane_offset_s_start[active_lo];
    lane_offset_val =
        lane_offsets_.lane_offset_a[active_lo] +
        (ds_lo * (lane_offsets_.lane_offset_b[active_lo] +
                  ds_lo * (lane_offsets_.lane_offset_c[active_lo] + ds_lo * lane_offsets_.lane_offset_d[active_lo])));
  }

  t_left += lane_offset_val;
  t_right += lane_offset_val;
}

auto CompiledPhysicsModel::LaneToRoad(LanePose pose, QueryContext& /*ctx*/) const noexcept -> RoadPose {
  auto lane_idx = static_cast<uint32_t>(pose.lane);
  if (lane_idx >= lanes_.lane_original_id.size()) {
    return RoadPose{};
  }

  double s = pose.s;
  int target_id = lanes_.lane_original_id[lane_idx];
  RoadId road_id = lanes_.lane_road_id[lane_idx];
  auto road_idx = static_cast<uint32_t>(road_id);
  uint32_t sec_idx = lanes_.lane_section_idx[lane_idx];

  // 1. Compute cumulative inner boundary width
  double inner_boundary_t = 0.0;
  uint32_t first_lane_in_sec = lane_sections_.section_first_lane_idx[sec_idx];
  uint32_t lane_cnt_in_sec = lane_sections_.section_lane_count[sec_idx];

  for (uint32_t i = 0; i < lane_cnt_in_sec; ++i) {
    uint32_t other_idx = first_lane_in_sec + i;
    int other_id = lanes_.lane_original_id[other_idx];
    if (target_id > 0) {
      if (other_id > 0 && other_id < target_id) {
        inner_boundary_t += LaneWidth(static_cast<LaneId>(other_idx), s);
      }
    } else if (target_id < 0) {
      if (other_id < 0 && other_id > target_id) {
        inner_boundary_t += LaneWidth(static_cast<LaneId>(other_idx), s);
      }
    }
  }

  if (target_id < 0) {
    inner_boundary_t = -inner_boundary_t;
  }

  // 2. Target lane width
  double w_target = LaneWidth(pose.lane, s);

  // 3. Center line t of the lane
  double t_center = 0.0;
  if (target_id > 0) {
    t_center = inner_boundary_t + (0.5 * w_target);
  } else if (target_id < 0) {
    t_center = inner_boundary_t - (0.5 * w_target);
  }

  double road_t = t_center + pose.t;

  // 4. Add road-level laneOffset
  double lane_offset_val = 0.0;
  uint32_t lo_first = lane_offsets_.road_lane_offset_first_idx[road_idx];
  uint32_t lo_count = lane_offsets_.road_lane_offset_count[road_idx];
  if (lo_count > 0) {
    uint32_t active_lo = lo_first;
    for (uint32_t i = 0; i < lo_count; ++i) {
      uint32_t cur_lo = lo_first + i;
      if (s >= lane_offsets_.lane_offset_s_start[cur_lo]) {
        active_lo = cur_lo;
      } else {
        break;
      }
    }
    double ds_lo = s - lane_offsets_.lane_offset_s_start[active_lo];
    lane_offset_val = lane_offsets_.lane_offset_a[active_lo] + (lane_offsets_.lane_offset_b[active_lo] * ds_lo) +
                      (lane_offsets_.lane_offset_c[active_lo] * ds_lo * ds_lo) +
                      (lane_offsets_.lane_offset_d[active_lo] * ds_lo * ds_lo * ds_lo);
  }
  road_t += lane_offset_val;

  // 5. Evaluate lane height offset
  double h_inner = 0.0;
  double h_outer = 0.0;
  uint32_t h_first = lanes_.lane_first_height_idx[lane_idx];
  uint32_t h_count = lanes_.lane_height_count[lane_idx];
  if (h_count > 0) {
    uint32_t active_h = h_first;
    for (uint32_t i = 0; i < h_count; ++i) {
      uint32_t cur_h = h_first + i;
      if (s >= lane_heights_.lane_height_s_start[cur_h]) {
        active_h = cur_h;
      } else {
        break;
      }
    }
    h_inner = lane_heights_.lane_height_inner[active_h];
    h_outer = lane_heights_.lane_height_outer[active_h];
  }

  double f = 0.0;
  if (w_target > 0.0) {
    if (target_id > 0) {
      f = 0.5 + (pose.t / w_target);
    } else if (target_id < 0) {
      f = 0.5 - (pose.t / w_target);
    }
  }
  f = std::clamp(f, 0.0, 1.0);
  double h_offset = h_inner + (f * (h_outer - h_inner));
  double road_h = pose.h + h_offset;

  RoadPose road_pose;
  road_pose.s = s;
  road_pose.t = road_t;
  road_pose.h = road_h;
  road_pose.heading = pose.heading;
  road_pose.pitch = pose.pitch;
  road_pose.roll = pose.roll;
  road_pose.road = road_id;
  return road_pose;
}

auto CompiledPhysicsModel::RoadCount() const noexcept -> std::size_t { return road_string_ids_.size(); }

auto CompiledPhysicsModel::RoadIdFromString(std::string_view original_id) const noexcept -> std::optional<RoadId> {
  auto find_it = std::ranges::find(road_string_ids_, original_id);
  if (find_it != road_string_ids_.end()) {
    return static_cast<RoadId>(std::distance(road_string_ids_.begin(), find_it));
  }
  return std::nullopt;
}

auto CompiledPhysicsModel::OriginalRoadId(RoadId road_id) const noexcept -> std::string_view {
  auto idx = static_cast<uint32_t>(road_id);
  if (idx < road_string_ids_.size()) {
    return road_string_ids_[idx];
  }
  return "";
}

auto CompiledPhysicsModel::RoadLength(RoadId road_id) const noexcept -> double {
  auto idx = static_cast<uint32_t>(road_id);
  if (idx < road_lengths_.size()) {
    return road_lengths_[idx];
  }
  return 0.0;
}

auto CompiledPhysicsModel::LaneCount() const noexcept -> std::size_t { return lanes_.lane_original_id.size(); }

auto CompiledPhysicsModel::LaneRoad(LaneId lane_id) const noexcept -> RoadId {
  auto idx = static_cast<uint32_t>(lane_id);
  if (idx < lanes_.lane_road_id.size()) {
    return lanes_.lane_road_id[idx];
  }
  return RoadId{0};
}

auto CompiledPhysicsModel::OriginalLaneId(LaneId lane_id) const noexcept -> int {
  auto idx = static_cast<uint32_t>(lane_id);
  if (idx < lanes_.lane_original_id.size()) {
    return lanes_.lane_original_id[idx];
  }
  return 0;
}

auto CompiledPhysicsModel::LaneWidth(LaneId lane_id, double s_coord) const noexcept -> double {
  auto idx = static_cast<uint32_t>(lane_id);
  if (idx >= lanes_.lane_original_id.size()) {
    return 0.0;
  }
  uint32_t w_first = lanes_.lane_first_width_idx[idx];
  uint32_t w_count = lanes_.lane_width_count[idx];
  if (w_count == 0) {
    return 0.0;
  }

  uint32_t active_idx = w_first;
  for (uint32_t i = 0; i < w_count; ++i) {
    uint32_t cur_idx = w_first + i;
    if (s_coord >= lane_widths_.lane_width_s_start[cur_idx]) {
      active_idx = cur_idx;
    } else {
      break;
    }
  }

  double ds = s_coord - lane_widths_.lane_width_s_start[active_idx];
  return lane_widths_.lane_width_a[active_idx] + (lane_widths_.lane_width_b[active_idx] * ds) +
         (lane_widths_.lane_width_c[active_idx] * ds * ds) + (lane_widths_.lane_width_d[active_idx] * ds * ds * ds);
}

auto BuildCompiledPhysicsModel(const ast::AbstractSyntaxTree& map) -> CompiledPhysicsModel {
  CompiledPhysicsModel model;
  model.ref_line_ = ReferenceLine::Build(map);

  for (const auto& road : map.roads) {
    model.road_string_ids_.push_back(road.id);
    model.road_lengths_.push_back(road.length);

    // Elevation profile compilation
    {
      auto first_idx = 0U;
      auto count = 0U;
      std::vector<ast::Coefficient> coeffs;
      coeffs.reserve(road.elevation_profile.elevations.size());
      for (const auto& elev : road.elevation_profile.elevations) {
        coeffs.push_back({elev.s, elev.a, elev.b, elev.c, elev.d});
      }
      CompileCoefficients(coeffs, model.polynomials_, first_idx, count);
      model.elevation_.road_elevation_first_idx.push_back(first_idx);
      model.elevation_.road_elevation_count.push_back(count);
    }

    // Superelevation profile compilation
    {
      auto first_idx = 0U;
      auto count = 0U;
      std::vector<ast::Coefficient> coeffs;
      coeffs.reserve(road.lateral_profile.superelevations.size());
      for (const auto& super : road.lateral_profile.superelevations) {
        coeffs.push_back({super.s, super.a, super.b, super.c, super.d});
      }
      CompileCoefficients(coeffs, model.polynomials_, first_idx, count);
      model.elevation_.road_superelevation_first_idx.push_back(first_idx);
      model.elevation_.road_superelevation_count.push_back(count);
    }

    // Shape profile compilation
    {
      auto first_idx = static_cast<uint32_t>(model.shapes_.s.size());
      auto count = static_cast<uint32_t>(road.lateral_profile.shapes.size());
      for (const auto& shape : road.lateral_profile.shapes) {
        model.shapes_.s.push_back(shape.s);
        model.shapes_.t.push_back(shape.t);
        model.shapes_.a.push_back(shape.a);
        model.shapes_.b.push_back(shape.b);
        model.shapes_.c.push_back(shape.c);
        model.shapes_.d.push_back(shape.d);
      }
      model.shapes_.road_shape_first_idx.push_back(first_idx);
      model.shapes_.road_shape_count.push_back(count);
    }

    const auto& css_opt = road.lateral_profile.cross_section_surface;
    if (css_opt.has_value()) {
      const auto& css = *css_opt;

      model.road_css_.first_strip_idx.push_back(static_cast<uint32_t>(model.strips_.strip_id.size()));
      model.road_css_.strip_count.push_back(static_cast<uint32_t>(css.strips.size()));

      auto t_off_idx = 0U;
      auto t_off_cnt = 0U;
      CompileCoefficients(css.t_offset, model.polynomials_, t_off_idx, t_off_cnt);
      model.road_css_.t_offset_first_idx.push_back(t_off_idx);
      model.road_css_.t_offset_count.push_back(t_off_cnt);

      auto sorted_strips = css.strips;
      std::ranges::sort(sorted_strips, [](const auto& val_a, const auto& val_b) noexcept -> bool {
        return std::abs(val_a.id) < std::abs(val_b.id);
      });

      for (const auto& strip : sorted_strips) {
        model.strips_.strip_id.push_back(strip.id);
        model.strips_.is_relative.push_back(static_cast<uint8_t>(strip.mode == "relative"));

        auto first_idx = 0U;
        auto count = 0U;

        CompileCoefficients(strip.width, model.polynomials_, first_idx, count);
        model.strips_.width_first_idx.push_back(first_idx);
        model.strips_.width_count.push_back(count);

        CompileCoefficients(strip.constant, model.polynomials_, first_idx, count);
        model.strips_.c0_first_idx.push_back(first_idx);
        model.strips_.c0_count.push_back(count);

        CompileCoefficients(strip.linear, model.polynomials_, first_idx, count);
        model.strips_.c1_first_idx.push_back(first_idx);
        model.strips_.c1_count.push_back(count);

        CompileCoefficients(strip.quadratic, model.polynomials_, first_idx, count);
        model.strips_.c2_first_idx.push_back(first_idx);
        model.strips_.c2_count.push_back(count);

        CompileCoefficients(strip.cubic, model.polynomials_, first_idx, count);
        model.strips_.c3_first_idx.push_back(first_idx);
        model.strips_.c3_count.push_back(count);
      }
    } else {
      model.road_css_.first_strip_idx.push_back(0);
      model.road_css_.strip_count.push_back(0);
      model.road_css_.t_offset_first_idx.push_back(0);
      model.road_css_.t_offset_count.push_back(0);
    }

    // Lane offset profile compilation (road level)
    {
      auto first_idx = static_cast<uint32_t>(model.lane_offsets_.lane_offset_s_start.size());
      auto count = static_cast<uint32_t>(road.lanes.offsets.size());
      for (const auto& offset : road.lanes.offsets) {
        model.lane_offsets_.lane_offset_s_start.push_back(offset.s);
        model.lane_offsets_.lane_offset_a.push_back(offset.a);
        model.lane_offsets_.lane_offset_b.push_back(offset.b);
        model.lane_offsets_.lane_offset_c.push_back(offset.c);
        model.lane_offsets_.lane_offset_d.push_back(offset.d);
      }
      model.lane_offsets_.road_lane_offset_first_idx.push_back(first_idx);
      model.lane_offsets_.road_lane_offset_count.push_back(count);
    }

    // Lane sections compilation
    {
      auto road_sec_first = static_cast<uint32_t>(model.lane_sections_.section_s.size());
      auto road_sec_count = static_cast<uint32_t>(road.lanes.sections.size());

      for (const auto& section : road.lanes.sections) {
        model.lane_sections_.section_s.push_back(section.s);

        // Accumulate all lanes in this section: right, center, left.
        // We will sort them by ID ascending.
        std::vector<ast::Lane> sorted_section_lanes;
        sorted_section_lanes.reserve(section.right.size() + section.center.size() + section.left.size());
        for (const auto& lane : section.right) {
          sorted_section_lanes.push_back(lane);
        }
        for (const auto& lane : section.center) {
          sorted_section_lanes.push_back(lane);
        }
        for (const auto& lane : section.left) {
          sorted_section_lanes.push_back(lane);
        }

        // Sort by ID ascending
        std::ranges::sort(sorted_section_lanes, [](const auto& lhs_lane, const auto& rhs_lane) noexcept -> bool {
          return lhs_lane.id < rhs_lane.id;
        });

        auto section_first_lane = static_cast<uint32_t>(model.lanes_.lane_original_id.size());
        auto section_lane_count = static_cast<uint32_t>(sorted_section_lanes.size());

        model.lane_sections_.section_first_lane_idx.push_back(section_first_lane);
        model.lane_sections_.section_lane_count.push_back(section_lane_count);

        // Now compile each lane in this section
        for (const auto& lane : sorted_section_lanes) {
          model.lanes_.lane_original_id.push_back(lane.id);
          model.lanes_.lane_road_id.push_back(static_cast<RoadId>(model.road_string_ids_.size() - 1));
          model.lanes_.lane_section_idx.push_back(static_cast<uint32_t>(model.lane_sections_.section_s.size() - 1));

          // Width polynomials for this lane
          auto w_first = static_cast<uint32_t>(model.lane_widths_.lane_width_s_start.size());
          auto w_count = static_cast<uint32_t>(lane.widths.size());
          for (const auto& width_poly : lane.widths) {
            // Note: sOffset is relative to section start s.
            model.lane_widths_.lane_width_s_start.push_back(section.s + width_poly.s_offset);
            model.lane_widths_.lane_width_a.push_back(width_poly.a);
            model.lane_widths_.lane_width_b.push_back(width_poly.b);
            model.lane_widths_.lane_width_c.push_back(width_poly.c);
            model.lane_widths_.lane_width_d.push_back(width_poly.d);
          }
          model.lanes_.lane_first_width_idx.push_back(w_first);
          model.lanes_.lane_width_count.push_back(w_count);

          // Height polynomials for this lane
          auto h_first = static_cast<uint32_t>(model.lane_heights_.lane_height_s_start.size());
          auto h_count = static_cast<uint32_t>(lane.heights.size());
          for (const auto& height_poly : lane.heights) {
            // sOffset is relative to section start s.
            model.lane_heights_.lane_height_s_start.push_back(section.s + height_poly.s_offset);
            model.lane_heights_.lane_height_inner.push_back(height_poly.inner);
            model.lane_heights_.lane_height_outer.push_back(height_poly.outer);
          }
          model.lanes_.lane_first_height_idx.push_back(h_first);
          model.lanes_.lane_height_count.push_back(h_count);
        }
      }
      model.lane_sections_.road_section_first_idx.push_back(road_sec_first);
      model.lane_sections_.road_section_count.push_back(road_sec_count);
    }
  }

  // Global bounding volume hierarchy construction
  std::vector<double> road_max_t;
  road_max_t.reserve(map.roads.size());

  for (const auto& road : map.roads) {
    double max_road_t = 0.0;
    for (size_t sec_idx = 0; sec_idx < road.lanes.sections.size(); ++sec_idx) {
      const auto& section = road.lanes.sections[sec_idx];
      double sec_length = 0.0;
      if (sec_idx + 1 < road.lanes.sections.size()) {
        sec_length = road.lanes.sections[sec_idx + 1].s - section.s;
      } else {
        sec_length = road.length - section.s;
      }

      constexpr int kSecSamples = 10;
      for (int i = 0; i <= kSecSamples; ++i) {
        double s_local = (static_cast<double>(i) / kSecSamples) * sec_length;
        double left_width = 0.0;
        double right_width = 0.0;

        for (const auto& lane : section.left) {
          left_width += EvaluateAstLaneWidth(lane, s_local);
        }
        for (const auto& lane : section.right) {
          right_width += EvaluateAstLaneWidth(lane, s_local);
        }

        // Evaluate road laneOffset
        double lane_offset_val = 0.0;
        if (!road.lanes.offsets.empty()) {
          const ast::LaneOffset* active = road.lanes.offsets.data();
          double s_road = section.s + s_local;
          for (const auto& offset : road.lanes.offsets) {
            if (s_road >= offset.s) {
              active = &offset;
            } else {
              break;
            }
          }
          double ds_offset = s_road - active->s;
          lane_offset_val = active->a + (ds_offset * (active->b + (ds_offset * (active->c + (ds_offset * active->d)))));
        }

        max_road_t = std::max(max_road_t, left_width + std::abs(lane_offset_val));
        max_road_t = std::max(max_road_t, right_width + std::abs(lane_offset_val));
      }
    }
    constexpr double kRoadWidthSafetyBuffer = 0.1;
    max_road_t += kRoadWidthSafetyBuffer;
    road_max_t.push_back(max_road_t);
  }

  std::vector<BoundingVolumeHierarchy::PrimitiveInfo> temp_primitives;
  std::vector<Aabb> temp_aabbs;

  auto num_roads = static_cast<uint32_t>(model.road_lengths_.size());
  for (uint32_t road_idx = 0; road_idx < num_roads; ++road_idx) {
    auto [first_seg, seg_count] = model.ref_line_.GetRoadSegments(static_cast<RoadId>(road_idx));
    double inflation = road_max_t[road_idx];
    for (uint32_t i = 0; i < seg_count; ++i) {
      uint32_t seg_idx = first_seg + i;
      temp_primitives.push_back(BoundingVolumeHierarchy::PrimitiveInfo{.road_idx = road_idx, .segment_idx = seg_idx});
      auto aabb = model.ref_line_.ComputeSegmentAabb(seg_idx, inflation);
      temp_aabbs.push_back(aabb);
    }
  }

  if (!temp_primitives.empty()) {
    std::vector<uint32_t> prim_indices(temp_primitives.size());
    for (uint32_t i = 0; i < prim_indices.size(); ++i) {
      prim_indices[i] = i;
    }
    model.bounding_volume_hierarchy_ = BoundingVolumeHierarchy::Build(prim_indices, temp_primitives, temp_aabbs);
  }

  return model;
}

}  // namespace strada::cpm
