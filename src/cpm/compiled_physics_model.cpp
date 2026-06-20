#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <strada/cpm/aligned_allocator.hpp>
#include <strada/cpm/compiled_physics_model.hpp>

#include "geometry_math.hpp"
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

auto FindSegmentIndex(const ReferenceLineSoA& ref_line, RoadPose pose, QueryContext& ctx, uint32_t first_seg,
                      uint32_t seg_count) noexcept -> uint32_t {
  uint32_t seg_idx = first_seg;
  bool hit = false;

  if (ctx.last_road == pose.road && ctx.last_segment_idx.has_value()) {
    uint32_t last_idx = *ctx.last_segment_idx;
    if (last_idx >= first_seg && last_idx < first_seg + seg_count) {
      double s_start = ref_line.s_offset[last_idx];
      double s_end = s_start + ref_line.length[last_idx];
      if (pose.s >= s_start && pose.s < s_end) [[likely]] {
        seg_idx = last_idx;
        hit = true;
      }
    }
  }

  if (!hit) [[unlikely]] {
    for (uint32_t i = 0; i < seg_count; ++i) {
      uint32_t idx = first_seg + i;
      if (pose.s >= ref_line.s_offset[idx]) {
        seg_idx = idx;
      } else {
        break;
      }
    }
    ctx.last_road = pose.road;
    ctx.last_segment_idx = seg_idx;
  }
  return seg_idx;
}

void EvaluateReferenceLine(const ReferenceLineSoA& ref_line, const AlignedVector<double>& arc_curvature,
                           uint32_t seg_idx, double s_coord, double& ref_x, double& ref_y,
                           double& tangent_hdg) noexcept {
  double ds_val = s_coord - ref_line.s_offset[seg_idx];
  ref_x = ref_line.x[seg_idx];
  ref_y = ref_line.y[seg_idx];
  double ref_hdg = ref_line.hdg[seg_idx];
  tangent_hdg = ref_hdg;

  GeometryType type = ref_line.type[seg_idx];
  uint32_t type_idx = ref_line.type_index[seg_idx];

  if (type == GeometryType::kLine) {
    ref_x += ds_val * std::cos(ref_hdg);
    ref_y += ds_val * std::sin(ref_hdg);
  } else if (type == GeometryType::kArc) {
    double curvature = arc_curvature[type_idx];
    if (std::abs(curvature) > kCurvatureThreshold) {
      tangent_hdg += curvature * ds_val;
      ref_x += (1.0 / curvature) * (std::sin(tangent_hdg) - std::sin(ref_hdg));
      ref_y -= (1.0 / curvature) * (std::cos(tangent_hdg) - std::cos(ref_hdg));
    } else {
      ref_x += ds_val * std::cos(ref_hdg);
      ref_y += ds_val * std::sin(ref_hdg);
    }
  } else if (type == GeometryType::kSpiral) {
    double curv_start = ref_line.spiral_curv_start[seg_idx];
    double curv_end = ref_line.spiral_curv_end[seg_idx];
    double length = ref_line.length[seg_idx];
    double lambda = (length > 0.0) ? ((curv_end - curv_start) / length) : 0.0;

    tangent_hdg += (curv_start * ds_val) + (0.5 * (lambda * (ds_val * ds_val)));

    double param_a = lambda * ds_val * ds_val;
    double param_b = curv_start * ds_val;
    auto [local_x, local_y] = EvaluateClothoidIntegrals(param_a, param_b);

    ref_x += ds_val * (std::cos(ref_hdg) * local_x - std::sin(ref_hdg) * local_y);
    ref_y += ds_val * (std::sin(ref_hdg) * local_x + std::cos(ref_hdg) * local_y);
  } else if (type == GeometryType::kParamPoly3) {
    double a_u = ref_line.pp3_a_u[seg_idx];
    double b_u = ref_line.pp3_b_u[seg_idx];
    double c_u = ref_line.pp3_c_u[seg_idx];
    double d_u = ref_line.pp3_d_u[seg_idx];
    double a_v = ref_line.pp3_a_v[seg_idx];
    double b_v = ref_line.pp3_b_v[seg_idx];
    double c_v = ref_line.pp3_c_v[seg_idx];
    double d_v = ref_line.pp3_d_v[seg_idx];
    uint8_t p_range = ref_line.pp3_p_range[seg_idx];

    double length = ref_line.length[seg_idx];
    double p_val = ds_val;
    if (p_range == 0U) {
      p_val = (length > 0.0) ? (ds_val / length) : 0.0;
    }

    double u_p = a_u + (p_val * (b_u + (p_val * (c_u + (d_u * p_val)))));
    double v_p = a_v + (p_val * (b_v + (p_val * (c_v + (d_v * p_val)))));

    double du_dp = b_u + (p_val * ((kPolyCoeff2 * c_u) + (kPolyCoeff3 * d_u * p_val)));
    double dv_dp = b_v + (p_val * ((kPolyCoeff2 * c_v) + (kPolyCoeff3 * d_v * p_val)));

    double cos_hdg = std::cos(ref_hdg);
    double sin_hdg = std::sin(ref_hdg);

    ref_x += (u_p * cos_hdg) - (v_p * sin_hdg);
    ref_y += (u_p * sin_hdg) + (v_p * cos_hdg);
    tangent_hdg += std::atan2(dv_dp, du_dp);
  }
}

void EvaluateNaturalOrientationAndElev(const PolynomialsSoA& polynomials,
                                       const std::vector<uint32_t>& road_elevation_first_idx,
                                       const std::vector<uint32_t>& road_elevation_count,
                                       const std::vector<uint32_t>& road_superelevation_first_idx,
                                       const std::vector<uint32_t>& road_superelevation_count,
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

}  // namespace

[[gnu::hot]] auto CompiledPhysicsModel::RoadToInertial(RoadPose pose, QueryContext& ctx) const noexcept
    -> InertialPose {
  auto road_idx = static_cast<uint32_t>(pose.road);
  if (road_idx >= road_ref_line_count_.size()) {
    return InertialPose{};
  }

  uint32_t first_seg = road_ref_line_first_idx_[road_idx];
  uint32_t seg_count = road_ref_line_count_[road_idx];
  if (seg_count == 0) {
    return InertialPose{};
  }

  // 1. Find segment index
  uint32_t seg_idx = FindSegmentIndex(ref_line_, pose, ctx, first_seg, seg_count);

  // 2. Evaluate reference line
  double ref_x = 0.0;
  double ref_y = 0.0;
  double tangent_hdg = 0.0;
  EvaluateReferenceLine(ref_line_, arc_curvature_, seg_idx, pose.s, ref_x, ref_y, tangent_hdg);

  // 3. Evaluate natural pitch, roll, and elevation
  double elev = 0.0;
  double natural_pitch = 0.0;
  double natural_roll = 0.0;
  EvaluateNaturalOrientationAndElev(polynomials_, road_elevation_first_idx_, road_elevation_count_,
                                    road_superelevation_first_idx_, road_superelevation_count_, road_css_, road_idx,
                                    pose.s, elev, natural_pitch, natural_roll);

  // 4. Cross section surface height offset
  double h_surf = 0.0;
  EvaluateCrossSectionSurfaceOffset(polynomials_, strips_, road_css_, road_idx, pose.s, pose.t, h_surf);

  // 5. Position composition
  Matrix3x3 r_road = EulerToMatrix(tangent_hdg, natural_pitch, natural_roll);

  double local_t = pose.t;
  double local_h = pose.h + h_surf;

  double offset_x = (r_road[0][1] * local_t) + (r_road[0][2] * local_h);
  double offset_y = (r_road[1][1] * local_t) + (r_road[1][2] * local_h);
  double offset_z = (r_road[2][1] * local_t) + (r_road[2][2] * local_h);

  InertialPose inertial_pose;
  inertial_pose.x = ref_x + offset_x;
  inertial_pose.y = ref_y + offset_y;
  inertial_pose.z = elev + offset_z;

  // Composed orientation composition
  Matrix3x3 r_offset = EulerToMatrix(pose.heading, pose.pitch, pose.roll);
  Matrix3x3 r_inertial = ComposeRotations(r_road, r_offset);

  EulerAngles euler_angles = MatrixToEuler(r_inertial);
  inertial_pose.heading = euler_angles.heading;
  inertial_pose.pitch = euler_angles.pitch;
  inertial_pose.roll = euler_angles.roll;

  return inertial_pose;
}

auto CompiledPhysicsModel::LaneToInertial(LanePose /*pose*/, QueryContext& /*ctx*/) const noexcept -> InertialPose {
  (void)road_lengths_;
  return InertialPose{};
}

auto CompiledPhysicsModel::InertialToRoad(InertialPosition /*position*/, QueryContext& /*ctx*/) const noexcept
    -> std::optional<RoadPose> {
  (void)road_lengths_;
  return std::nullopt;
}

auto CompiledPhysicsModel::InertialToLane(InertialPosition /*position*/, QueryContext& /*ctx*/) const noexcept
    -> std::optional<LanePose> {
  (void)road_lengths_;
  return std::nullopt;
}

auto CompiledPhysicsModel::RoadToLane(RoadPose /*pose*/, QueryContext& /*ctx*/) const noexcept
    -> std::optional<LanePose> {
  (void)road_lengths_;
  return std::nullopt;
}

auto CompiledPhysicsModel::LaneToRoad(LanePose /*pose*/, QueryContext& /*ctx*/) const noexcept -> RoadPose {
  (void)road_lengths_;
  return RoadPose{};
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

auto CompiledPhysicsModel::LaneCount() const noexcept -> std::size_t {
  (void)road_lengths_;
  return 0;
}

auto CompiledPhysicsModel::LaneRoad(LaneId /*id*/) const noexcept -> RoadId {
  (void)road_lengths_;
  return RoadId{0};
}

auto CompiledPhysicsModel::OriginalLaneId(LaneId /*id*/) const noexcept -> int {
  (void)road_lengths_;
  return 0;
}

auto CompiledPhysicsModel::LaneWidth(LaneId /*id*/, double /*s_coord*/) const noexcept -> double {
  (void)road_lengths_;
  return 0.0;
}

auto BuildCompiledPhysicsModel(const ast::AbstractSyntaxTree& map) -> CompiledPhysicsModel {
  CompiledPhysicsModel model;

  for (const auto& road : map.roads) {
    model.road_string_ids_.push_back(road.id);
    model.road_lengths_.push_back(road.length);

    // Reference line compilation
    model.road_ref_line_first_idx_.push_back(static_cast<uint32_t>(model.ref_line_.s_offset.size()));
    model.road_ref_line_count_.push_back(static_cast<uint32_t>(road.plan_view.size()));

    for (const auto& geom : road.plan_view) {
      model.ref_line_.s_offset.push_back(geom.s);
      model.ref_line_.length.push_back(geom.length);
      model.ref_line_.x.push_back(geom.x);
      model.ref_line_.y.push_back(geom.y);
      model.ref_line_.hdg.push_back(geom.hdg);

      if (std::holds_alternative<ast::Line>(geom.shape)) {
        model.ref_line_.type.push_back(GeometryType::kLine);
        model.ref_line_.type_index.push_back(0);
        model.ref_line_.spiral_curv_start.push_back(0.0);
        model.ref_line_.spiral_curv_end.push_back(0.0);
        model.ref_line_.pp3_a_u.push_back(0.0);
        model.ref_line_.pp3_b_u.push_back(0.0);
        model.ref_line_.pp3_c_u.push_back(0.0);
        model.ref_line_.pp3_d_u.push_back(0.0);
        model.ref_line_.pp3_a_v.push_back(0.0);
        model.ref_line_.pp3_b_v.push_back(0.0);
        model.ref_line_.pp3_c_v.push_back(0.0);
        model.ref_line_.pp3_d_v.push_back(0.0);
        model.ref_line_.pp3_p_range.push_back(0);
      } else if (std::holds_alternative<ast::Arc>(geom.shape)) {
        model.ref_line_.type.push_back(GeometryType::kArc);
        model.ref_line_.type_index.push_back(static_cast<uint32_t>(model.arc_curvature_.size()));
        model.arc_curvature_.push_back(std::get<ast::Arc>(geom.shape).curvature);
        model.ref_line_.spiral_curv_start.push_back(0.0);
        model.ref_line_.spiral_curv_end.push_back(0.0);
        model.ref_line_.pp3_a_u.push_back(0.0);
        model.ref_line_.pp3_b_u.push_back(0.0);
        model.ref_line_.pp3_c_u.push_back(0.0);
        model.ref_line_.pp3_d_u.push_back(0.0);
        model.ref_line_.pp3_a_v.push_back(0.0);
        model.ref_line_.pp3_b_v.push_back(0.0);
        model.ref_line_.pp3_c_v.push_back(0.0);
        model.ref_line_.pp3_d_v.push_back(0.0);
        model.ref_line_.pp3_p_range.push_back(0);
      } else if (std::holds_alternative<ast::Spiral>(geom.shape)) {
        model.ref_line_.type.push_back(GeometryType::kSpiral);
        model.ref_line_.type_index.push_back(0);
        const auto& spiral = std::get<ast::Spiral>(geom.shape);
        model.ref_line_.spiral_curv_start.push_back(spiral.curv_start);
        model.ref_line_.spiral_curv_end.push_back(spiral.curv_end);
        model.ref_line_.pp3_a_u.push_back(0.0);
        model.ref_line_.pp3_b_u.push_back(0.0);
        model.ref_line_.pp3_c_u.push_back(0.0);
        model.ref_line_.pp3_d_u.push_back(0.0);
        model.ref_line_.pp3_a_v.push_back(0.0);
        model.ref_line_.pp3_b_v.push_back(0.0);
        model.ref_line_.pp3_c_v.push_back(0.0);
        model.ref_line_.pp3_d_v.push_back(0.0);
        model.ref_line_.pp3_p_range.push_back(0);
      } else if (std::holds_alternative<ast::Poly3>(geom.shape)) {
        model.ref_line_.type.push_back(GeometryType::kParamPoly3);
        model.ref_line_.type_index.push_back(0);
        model.ref_line_.spiral_curv_start.push_back(0.0);
        model.ref_line_.spiral_curv_end.push_back(0.0);
        const auto& poly = std::get<ast::Poly3>(geom.shape);
        ast::ParamPoly3 param = ConvertPoly3ToParamPoly3(geom.length, poly.a, poly.b, poly.c, poly.d);
        model.ref_line_.pp3_a_u.push_back(param.a_u);
        model.ref_line_.pp3_b_u.push_back(param.b_u);
        model.ref_line_.pp3_c_u.push_back(param.c_u);
        model.ref_line_.pp3_d_u.push_back(param.d_u);
        model.ref_line_.pp3_a_v.push_back(param.a_v);
        model.ref_line_.pp3_b_v.push_back(param.b_v);
        model.ref_line_.pp3_c_v.push_back(param.c_v);
        model.ref_line_.pp3_d_v.push_back(param.d_v);
        model.ref_line_.pp3_p_range.push_back(1);
      } else if (std::holds_alternative<ast::ParamPoly3>(geom.shape)) {
        model.ref_line_.type.push_back(GeometryType::kParamPoly3);
        model.ref_line_.type_index.push_back(0);
        model.ref_line_.spiral_curv_start.push_back(0.0);
        model.ref_line_.spiral_curv_end.push_back(0.0);
        const auto& param = std::get<ast::ParamPoly3>(geom.shape);
        model.ref_line_.pp3_a_u.push_back(param.a_u);
        model.ref_line_.pp3_b_u.push_back(param.b_u);
        model.ref_line_.pp3_c_u.push_back(param.c_u);
        model.ref_line_.pp3_d_u.push_back(param.d_u);
        model.ref_line_.pp3_a_v.push_back(param.a_v);
        model.ref_line_.pp3_b_v.push_back(param.b_v);
        model.ref_line_.pp3_c_v.push_back(param.c_v);
        model.ref_line_.pp3_d_v.push_back(param.d_v);
        model.ref_line_.pp3_p_range.push_back(param.p_range == ast::PRange::kArcLength ? 1 : 0);
      }
    }

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
      model.road_elevation_first_idx_.push_back(first_idx);
      model.road_elevation_count_.push_back(count);
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
      model.road_superelevation_first_idx_.push_back(first_idx);
      model.road_superelevation_count_.push_back(count);
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
  }

  return model;
}

}  // namespace strada::cpm
