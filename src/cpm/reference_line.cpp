#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <strada/cpm/geometry_math.hpp>
#include <strada/cpm/reference_line.hpp>

namespace strada::cpm {

namespace {

constexpr double kCurvatureThreshold = 1e-12;
constexpr double kPolyCoeff2 = 2.0;
constexpr double kPolyCoeff3 = 3.0;

}  // namespace

ReferenceLine::ReferenceLine(const ast::AbstractSyntaxTree& map) {
  for (const auto& road : map.roads) {
    road_ref_line_first_idx_.push_back(static_cast<std::uint32_t>(s_offset_.size()));
    road_ref_line_count_.push_back(static_cast<std::uint32_t>(road.plan_view.size()));

    for (const auto& geom : road.plan_view) {
      s_offset_.push_back(geom.s);
      length_.push_back(geom.length);
      x_.push_back(geom.x);
      y_.push_back(geom.y);
      hdg_.push_back(geom.hdg);

      if (std::holds_alternative<ast::Line>(geom.shape)) {
        type_.push_back(GeometryType::kLine);
        type_index_.push_back(0);
        spiral_curv_start_.push_back(0.0);
        spiral_curv_end_.push_back(0.0);
        pp3_a_u_.push_back(0.0);
        pp3_b_u_.push_back(0.0);
        pp3_c_u_.push_back(0.0);
        pp3_d_u_.push_back(0.0);
        pp3_a_v_.push_back(0.0);
        pp3_b_v_.push_back(0.0);
        pp3_c_v_.push_back(0.0);
        pp3_d_v_.push_back(0.0);
        pp3_p_range_.push_back(0);
      } else if (std::holds_alternative<ast::Arc>(geom.shape)) {
        type_.push_back(GeometryType::kArc);
        type_index_.push_back(static_cast<std::uint32_t>(arc_curvature_.size()));
        arc_curvature_.push_back(std::get<ast::Arc>(geom.shape).curvature);
        spiral_curv_start_.push_back(0.0);
        spiral_curv_end_.push_back(0.0);
        pp3_a_u_.push_back(0.0);
        pp3_b_u_.push_back(0.0);
        pp3_c_u_.push_back(0.0);
        pp3_d_u_.push_back(0.0);
        pp3_a_v_.push_back(0.0);
        pp3_b_v_.push_back(0.0);
        pp3_c_v_.push_back(0.0);
        pp3_d_v_.push_back(0.0);
        pp3_p_range_.push_back(0);
      } else if (std::holds_alternative<ast::Spiral>(geom.shape)) {
        type_.push_back(GeometryType::kSpiral);
        type_index_.push_back(0);
        const auto& spiral = std::get<ast::Spiral>(geom.shape);
        spiral_curv_start_.push_back(spiral.curv_start);
        spiral_curv_end_.push_back(spiral.curv_end);
        pp3_a_u_.push_back(0.0);
        pp3_b_u_.push_back(0.0);
        pp3_c_u_.push_back(0.0);
        pp3_d_u_.push_back(0.0);
        pp3_a_v_.push_back(0.0);
        pp3_b_v_.push_back(0.0);
        pp3_c_v_.push_back(0.0);
        pp3_d_v_.push_back(0.0);
        pp3_p_range_.push_back(0);
      } else if (std::holds_alternative<ast::Poly3>(geom.shape)) {
        type_.push_back(GeometryType::kParamPoly3);
        type_index_.push_back(0);
        spiral_curv_start_.push_back(0.0);
        spiral_curv_end_.push_back(0.0);
        const auto& poly = std::get<ast::Poly3>(geom.shape);
        const ast::ParamPoly3 param = ConvertPoly3ToParamPoly3(geom.length, poly.a, poly.b, poly.c, poly.d);
        pp3_a_u_.push_back(param.a_u);
        pp3_b_u_.push_back(param.b_u);
        pp3_c_u_.push_back(param.c_u);
        pp3_d_u_.push_back(param.d_u);
        pp3_a_v_.push_back(param.a_v);
        pp3_b_v_.push_back(param.b_v);
        pp3_c_v_.push_back(param.c_v);
        pp3_d_v_.push_back(param.d_v);
        pp3_p_range_.push_back(1);
      } else if (std::holds_alternative<ast::ParamPoly3>(geom.shape)) {
        type_.push_back(GeometryType::kParamPoly3);
        type_index_.push_back(0);
        spiral_curv_start_.push_back(0.0);
        spiral_curv_end_.push_back(0.0);
        const auto& param = std::get<ast::ParamPoly3>(geom.shape);
        pp3_a_u_.push_back(param.a_u);
        pp3_b_u_.push_back(param.b_u);
        pp3_c_u_.push_back(param.c_u);
        pp3_d_u_.push_back(param.d_u);
        pp3_a_v_.push_back(param.a_v);
        pp3_b_v_.push_back(param.b_v);
        pp3_c_v_.push_back(param.c_v);
        pp3_d_v_.push_back(param.d_v);
        pp3_p_range_.push_back(param.p_range == ast::PRange::kArcLength ? 1 : 0);
      }
    }
  }
}

auto ReferenceLine::Evaluate(std::uint32_t seg_idx, double road_s) const noexcept -> ReferenceLinePoint {
  const double ds_val = road_s - s_offset_[seg_idx];
  double ref_x = x_[seg_idx];
  double ref_y = y_[seg_idx];
  const double ref_hdg = hdg_[seg_idx];
  double tangent_hdg = ref_hdg;

  const GeometryType type = type_[seg_idx];
  const std::uint32_t type_idx = type_index_[seg_idx];

  if (type == GeometryType::kLine) {
    ref_x += ds_val * std::cos(ref_hdg);
    ref_y += ds_val * std::sin(ref_hdg);
  } else if (type == GeometryType::kArc) {
    const double curvature = arc_curvature_[type_idx];
    if (std::abs(curvature) > kCurvatureThreshold) {
      tangent_hdg += curvature * ds_val;
      ref_x += (1.0 / curvature) * (std::sin(tangent_hdg) - std::sin(ref_hdg));
      ref_y -= (1.0 / curvature) * (std::cos(tangent_hdg) - std::cos(ref_hdg));
    } else {
      ref_x += ds_val * std::cos(ref_hdg);
      ref_y += ds_val * std::sin(ref_hdg);
    }
  } else if (type == GeometryType::kSpiral) {
    const double curv_start = spiral_curv_start_[seg_idx];
    const double curv_end = spiral_curv_end_[seg_idx];
    const double length = length_[seg_idx];
    const double lambda = (length > 0.0) ? ((curv_end - curv_start) / length) : 0.0;

    tangent_hdg += (curv_start * ds_val) + (0.5 * (lambda * (ds_val * ds_val)));

    const double param_a = lambda * ds_val * ds_val;
    const double param_b = curv_start * ds_val;
    auto [local_x, local_y] = EvaluateClothoidIntegrals(param_a, param_b);

    ref_x += ds_val * (std::cos(ref_hdg) * local_x - std::sin(ref_hdg) * local_y);
    ref_y += ds_val * (std::sin(ref_hdg) * local_x + std::cos(ref_hdg) * local_y);
  } else if (type == GeometryType::kParamPoly3) {
    const double au = pp3_a_u_[seg_idx];
    const double bu = pp3_b_u_[seg_idx];
    const double cu = pp3_c_u_[seg_idx];
    const double du = pp3_d_u_[seg_idx];
    const double av = pp3_a_v_[seg_idx];
    const double bv = pp3_b_v_[seg_idx];
    const double cv = pp3_c_v_[seg_idx];
    const double dv = pp3_d_v_[seg_idx];
    const std::uint8_t p_range = pp3_p_range_[seg_idx];

    const double length = length_[seg_idx];
    double p_val = ds_val;
    if (p_range == 0U) {
      p_val = (length > 0.0) ? (ds_val / length) : 0.0;
    }

    const double up = au + (p_val * (bu + (p_val * (cu + (du * p_val)))));
    const double vp = av + (p_val * (bv + (p_val * (cv + (dv * p_val)))));

    const double du_dp = bu + (p_val * ((kPolyCoeff2 * cu) + (kPolyCoeff3 * du * p_val)));
    const double dv_dp = bv + (p_val * ((kPolyCoeff2 * cv) + (kPolyCoeff3 * dv * p_val)));

    const double cos_hdg = std::cos(ref_hdg);
    const double sin_hdg = std::sin(ref_hdg);

    ref_x += (up * cos_hdg) - (vp * sin_hdg);
    ref_y += (up * sin_hdg) + (vp * cos_hdg);
    tangent_hdg += std::atan2(dv_dp, du_dp);
  }

  return ReferenceLinePoint{.x = ref_x, .y = ref_y, .heading = tangent_hdg};
}

auto ReferenceLine::Project(std::uint32_t seg_idx, double px, double py) const noexcept -> double {
  const double s_start = s_offset_[seg_idx];
  const double seg_length = length_[seg_idx];
  const GeometryType type = type_[seg_idx];
  const std::uint32_t type_idx = type_index_[seg_idx];

  if (type == GeometryType::kLine) {
    const double dx = px - x_[seg_idx];
    const double dy = py - y_[seg_idx];
    const double hdg = hdg_[seg_idx];
    const double ds = (dx * std::cos(hdg)) + (dy * std::sin(hdg));
    const double s_local = std::clamp(ds, 0.0, seg_length);
    return s_start + s_local;
  }
  if (type == GeometryType::kArc) {
    const double dx = px - x_[seg_idx];
    const double dy = py - y_[seg_idx];
    const double hdg = hdg_[seg_idx];
    const double curvature = arc_curvature_[type_idx];
    if (std::abs(curvature) < kCurvatureThreshold) {
      const double ds = (dx * std::cos(hdg)) + (dy * std::sin(hdg));
      const double s_local = std::clamp(ds, 0.0, seg_length);
      return s_start + s_local;
    }
    const double radius = 1.0 / curvature;
    const double center_x = x_[seg_idx] - (radius * std::sin(hdg));
    const double center_y = y_[seg_idx] + (radius * std::cos(hdg));
    const double qdx = px - center_x;
    const double qdy = py - center_y;
    const double angle_query = std::atan2(qdy, qdx);
    const double angle_start = std::atan2(y_[seg_idx] - center_y, x_[seg_idx] - center_x);
    double delta_angle = angle_query - angle_start;
    constexpr double two_pi = 2.0 * std::numbers::pi;
    if (curvature > 0.0) {
      while (delta_angle < 0.0) {
        delta_angle += two_pi;
      }
      while (delta_angle >= two_pi) {
        delta_angle -= two_pi;
      }
    } else {
      while (delta_angle > 0.0) {
        delta_angle -= two_pi;
      }
      while (delta_angle <= -two_pi) {
        delta_angle += two_pi;
      }
    }
    const double s_local = std::clamp(delta_angle / curvature, 0.0, seg_length);
    return s_start + s_local;
  }

  // Fallback to numerical solver for spirals and ParamPoly3
  constexpr int num_intervals = 10;
  double best_s = 0.0;
  double min_dist_sq = std::numeric_limits<double>::max();

  for (int i = 0; i <= num_intervals; ++i) {
    const double s_test = (static_cast<double>(i) / num_intervals) * seg_length;
    auto pt = Evaluate(seg_idx, s_start + s_test);
    const double dx = px - pt.x;
    const double dy = py - pt.y;
    const double dist_sq = (dx * dx) + (dy * dy);
    if (dist_sq < min_dist_sq) {
      min_dist_sq = dist_sq;
      best_s = s_test;
    }
  }

  double left_s = std::max(0.0, best_s - (seg_length / num_intervals));
  double right_s = std::min(seg_length, best_s + (seg_length / num_intervals));
  for (int iter = 0; iter < 30; ++iter) {
    const double m1 = left_s + ((right_s - left_s) / 3.0);
    const double m2 = right_s - ((right_s - left_s) / 3.0);
    auto pt1 = Evaluate(seg_idx, s_start + m1);
    auto pt2 = Evaluate(seg_idx, s_start + m2);
    const double dist1 = ((px - pt1.x) * (px - pt1.x)) + ((py - pt1.y) * (py - pt1.y));
    const double dist2 = ((px - pt2.x) * (px - pt2.x)) + ((py - pt2.y) * (py - pt2.y));
    if (dist1 < dist2) {
      right_s = m2;
    } else {
      left_s = m1;
    }
  }

  return s_start + (0.5 * (left_s + right_s));
}

auto ReferenceLine::FindSegmentIndex(RoadId road, double s_coord, QueryContext& ctx) const noexcept -> std::uint32_t {
  auto road_idx = static_cast<std::uint32_t>(road);
  const std::uint32_t first_seg = road_ref_line_first_idx_[road_idx];
  const std::uint32_t seg_count = road_ref_line_count_[road_idx];

  std::uint32_t seg_idx = first_seg;
  bool hit = false;

  if (ctx.last_road == road && ctx.last_segment_idx.has_value()) {
    const std::uint32_t last_idx = *ctx.last_segment_idx;
    if (last_idx >= first_seg && last_idx < first_seg + seg_count) {
      const double s_start = s_offset_[last_idx];
      const double s_end = s_start + length_[last_idx];
      if (s_coord >= s_start && s_coord < s_end) [[likely]] {
        seg_idx = last_idx;
        hit = true;
      }
    }
  }

  if (!hit) [[unlikely]] {
    for (std::uint32_t i = 0; i < seg_count; ++i) {
      const std::uint32_t idx = first_seg + i;
      if (s_coord >= s_offset_[idx]) {
        seg_idx = idx;
      } else {
        break;
      }
    }
    ctx.last_road = road;
    ctx.last_segment_idx = seg_idx;
  }
  return seg_idx;
}

auto ReferenceLine::GetRoadSegments(RoadId road) const noexcept -> std::pair<std::uint32_t, std::uint32_t> {
  auto road_idx = static_cast<std::uint32_t>(road);
  if (road_idx >= road_ref_line_first_idx_.size()) {
    return {0, 0};
  }
  return {road_ref_line_first_idx_[road_idx], road_ref_line_count_[road_idx]};
}

auto ReferenceLine::GetSegmentSStart(std::uint32_t seg_idx) const noexcept -> double { return s_offset_[seg_idx]; }

auto ReferenceLine::GetSegmentLength(std::uint32_t seg_idx) const noexcept -> double { return length_[seg_idx]; }

auto ReferenceLine::ComputeSegmentAabb(std::uint32_t seg_idx, double inflation) const noexcept -> Aabb {
  double min_x = std::numeric_limits<double>::max();
  double min_y = std::numeric_limits<double>::max();
  double max_x = -std::numeric_limits<double>::max();
  double max_y = -std::numeric_limits<double>::max();

  const double length = length_[seg_idx];
  const double s_start = s_offset_[seg_idx];

  int num_samples = 1;
  if (type_[seg_idx] != GeometryType::kLine) {
    num_samples = 32;
  }

  for (int idx = 0; idx <= num_samples; ++idx) {
    const double s_local = (static_cast<double>(idx) / num_samples) * length;
    auto pt = Evaluate(seg_idx, s_start + s_local);
    min_x = std::min(min_x, pt.x);
    min_y = std::min(min_y, pt.y);
    max_x = std::max(max_x, pt.x);
    max_y = std::max(max_y, pt.y);
  }

  Aabb bounds;
  bounds.min_x = min_x - inflation;
  bounds.min_y = min_y - inflation;
  bounds.max_x = max_x + inflation;
  bounds.max_y = max_y + inflation;
  return bounds;
}

auto ReferenceLine::TotalSegmentsCount() const noexcept -> std::size_t { return s_offset_.size(); }

}  // namespace strada::cpm
