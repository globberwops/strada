#include <algorithm>
#include <cmath>
#include <limits>
#include <strada/cpm/reference_line.hpp>

#include "geometry_math.hpp"

namespace strada::cpm {

namespace {

constexpr double kCurvatureThreshold = 1e-12;
constexpr double kPolyCoeff2 = 2.0;
constexpr double kPolyCoeff3 = 3.0;

}  // namespace

auto ReferenceLine::Build(const ast::AbstractSyntaxTree& map) -> ReferenceLine {
  ReferenceLine rl;

  for (const auto& road : map.roads) {
    rl.road_ref_line_first_idx_.push_back(static_cast<uint32_t>(rl.s_offset_.size()));
    rl.road_ref_line_count_.push_back(static_cast<uint32_t>(road.plan_view.size()));

    for (const auto& geom : road.plan_view) {
      rl.s_offset_.push_back(geom.s);
      rl.length_.push_back(geom.length);
      rl.x_.push_back(geom.x);
      rl.y_.push_back(geom.y);
      rl.hdg_.push_back(geom.hdg);

      if (std::holds_alternative<ast::Line>(geom.shape)) {
        rl.type_.push_back(GeometryType::kLine);
        rl.type_index_.push_back(0);
        rl.spiral_curv_start_.push_back(0.0);
        rl.spiral_curv_end_.push_back(0.0);
        rl.pp3_a_u_.push_back(0.0);
        rl.pp3_b_u_.push_back(0.0);
        rl.pp3_c_u_.push_back(0.0);
        rl.pp3_d_u_.push_back(0.0);
        rl.pp3_a_v_.push_back(0.0);
        rl.pp3_b_v_.push_back(0.0);
        rl.pp3_c_v_.push_back(0.0);
        rl.pp3_d_v_.push_back(0.0);
        rl.pp3_p_range_.push_back(0);
      } else if (std::holds_alternative<ast::Arc>(geom.shape)) {
        rl.type_.push_back(GeometryType::kArc);
        rl.type_index_.push_back(static_cast<uint32_t>(rl.arc_curvature_.size()));
        rl.arc_curvature_.push_back(std::get<ast::Arc>(geom.shape).curvature);
        rl.spiral_curv_start_.push_back(0.0);
        rl.spiral_curv_end_.push_back(0.0);
        rl.pp3_a_u_.push_back(0.0);
        rl.pp3_b_u_.push_back(0.0);
        rl.pp3_c_u_.push_back(0.0);
        rl.pp3_d_u_.push_back(0.0);
        rl.pp3_a_v_.push_back(0.0);
        rl.pp3_b_v_.push_back(0.0);
        rl.pp3_c_v_.push_back(0.0);
        rl.pp3_d_v_.push_back(0.0);
        rl.pp3_p_range_.push_back(0);
      } else if (std::holds_alternative<ast::Spiral>(geom.shape)) {
        rl.type_.push_back(GeometryType::kSpiral);
        rl.type_index_.push_back(0);
        const auto& spiral = std::get<ast::Spiral>(geom.shape);
        rl.spiral_curv_start_.push_back(spiral.curv_start);
        rl.spiral_curv_end_.push_back(spiral.curv_end);
        rl.pp3_a_u_.push_back(0.0);
        rl.pp3_b_u_.push_back(0.0);
        rl.pp3_c_u_.push_back(0.0);
        rl.pp3_d_u_.push_back(0.0);
        rl.pp3_a_v_.push_back(0.0);
        rl.pp3_b_v_.push_back(0.0);
        rl.pp3_c_v_.push_back(0.0);
        rl.pp3_d_v_.push_back(0.0);
        rl.pp3_p_range_.push_back(0);
      } else if (std::holds_alternative<ast::Poly3>(geom.shape)) {
        rl.type_.push_back(GeometryType::kParamPoly3);
        rl.type_index_.push_back(0);
        rl.spiral_curv_start_.push_back(0.0);
        rl.spiral_curv_end_.push_back(0.0);
        const auto& poly = std::get<ast::Poly3>(geom.shape);
        ast::ParamPoly3 param = ConvertPoly3ToParamPoly3(geom.length, poly.a, poly.b, poly.c, poly.d);
        rl.pp3_a_u_.push_back(param.a_u);
        rl.pp3_b_u_.push_back(param.b_u);
        rl.pp3_c_u_.push_back(param.c_u);
        rl.pp3_d_u_.push_back(param.d_u);
        rl.pp3_a_v_.push_back(param.a_v);
        rl.pp3_b_v_.push_back(param.b_v);
        rl.pp3_c_v_.push_back(param.c_v);
        rl.pp3_d_v_.push_back(param.d_v);
        rl.pp3_p_range_.push_back(1);
      } else if (std::holds_alternative<ast::ParamPoly3>(geom.shape)) {
        rl.type_.push_back(GeometryType::kParamPoly3);
        rl.type_index_.push_back(0);
        rl.spiral_curv_start_.push_back(0.0);
        rl.spiral_curv_end_.push_back(0.0);
        const auto& param = std::get<ast::ParamPoly3>(geom.shape);
        rl.pp3_a_u_.push_back(param.a_u);
        rl.pp3_b_u_.push_back(param.b_u);
        rl.pp3_c_u_.push_back(param.c_u);
        rl.pp3_d_u_.push_back(param.d_u);
        rl.pp3_a_v_.push_back(param.a_v);
        rl.pp3_b_v_.push_back(param.b_v);
        rl.pp3_c_v_.push_back(param.c_v);
        rl.pp3_d_v_.push_back(param.d_v);
        rl.pp3_p_range_.push_back(param.p_range == ast::PRange::kArcLength ? 1 : 0);
      }
    }
  }

  return rl;
}

auto ReferenceLine::Evaluate(uint32_t seg_idx, double global_s) const noexcept -> ReferenceLinePoint {
  double ds_val = global_s - s_offset_[seg_idx];
  double ref_x = x_[seg_idx];
  double ref_y = y_[seg_idx];
  double ref_hdg = hdg_[seg_idx];
  double tangent_hdg = ref_hdg;

  GeometryType type = type_[seg_idx];
  uint32_t type_idx = type_index_[seg_idx];

  if (type == GeometryType::kLine) {
    ref_x += ds_val * std::cos(ref_hdg);
    ref_y += ds_val * std::sin(ref_hdg);
  } else if (type == GeometryType::kArc) {
    double curvature = arc_curvature_[type_idx];
    if (std::abs(curvature) > kCurvatureThreshold) {
      tangent_hdg += curvature * ds_val;
      ref_x += (1.0 / curvature) * (std::sin(tangent_hdg) - std::sin(ref_hdg));
      ref_y -= (1.0 / curvature) * (std::cos(tangent_hdg) - std::cos(ref_hdg));
    } else {
      ref_x += ds_val * std::cos(ref_hdg);
      ref_y += ds_val * std::sin(ref_hdg);
    }
  } else if (type == GeometryType::kSpiral) {
    double curv_start = spiral_curv_start_[seg_idx];
    double curv_end = spiral_curv_end_[seg_idx];
    double length = length_[seg_idx];
    double lambda = (length > 0.0) ? ((curv_end - curv_start) / length) : 0.0;

    tangent_hdg += (curv_start * ds_val) + (0.5 * (lambda * (ds_val * ds_val)));

    double param_a = lambda * ds_val * ds_val;
    double param_b = curv_start * ds_val;
    auto [local_x, local_y] = EvaluateClothoidIntegrals(param_a, param_b);

    ref_x += ds_val * (std::cos(ref_hdg) * local_x - std::sin(ref_hdg) * local_y);
    ref_y += ds_val * (std::sin(ref_hdg) * local_x + std::cos(ref_hdg) * local_y);
  } else if (type == GeometryType::kParamPoly3) {
    double a_u = pp3_a_u_[seg_idx];
    double b_u = pp3_b_u_[seg_idx];
    double c_u = pp3_c_u_[seg_idx];
    double d_u = pp3_d_u_[seg_idx];
    double a_v = pp3_a_v_[seg_idx];
    double b_v = pp3_b_v_[seg_idx];
    double c_v = pp3_c_v_[seg_idx];
    double d_v = pp3_d_v_[seg_idx];
    uint8_t p_range = pp3_p_range_[seg_idx];

    double length = length_[seg_idx];
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

  return ReferenceLinePoint{.x = ref_x, .y = ref_y, .heading = tangent_hdg};
}

auto ReferenceLine::Project(uint32_t seg_idx, double px, double py) const noexcept -> double {
  double s_start = s_offset_[seg_idx];
  double seg_length = length_[seg_idx];
  GeometryType type = type_[seg_idx];
  uint32_t type_idx = type_index_[seg_idx];

  if (type == GeometryType::kLine) {
    double dx = px - x_[seg_idx];
    double dy = py - y_[seg_idx];
    double hdg = hdg_[seg_idx];
    double ds = (dx * std::cos(hdg)) + (dy * std::sin(hdg));
    double s_local = std::clamp(ds, 0.0, seg_length);
    return s_start + s_local;
  } else if (type == GeometryType::kArc) {
    double dx = px - x_[seg_idx];
    double dy = py - y_[seg_idx];
    double hdg = hdg_[seg_idx];
    double curvature = arc_curvature_[type_idx];
    if (std::abs(curvature) < kCurvatureThreshold) {
      double ds = (dx * std::cos(hdg)) + (dy * std::sin(hdg));
      double s_local = std::clamp(ds, 0.0, seg_length);
      return s_start + s_local;
    } else {
      double radius = 1.0 / curvature;
      double center_x = x_[seg_idx] - (radius * std::sin(hdg));
      double center_y = y_[seg_idx] + (radius * std::cos(hdg));
      double qdx = px - center_x;
      double qdy = py - center_y;
      double angle_query = std::atan2(qdy, qdx);
      double angle_start = std::atan2(y_[seg_idx] - center_y, x_[seg_idx] - center_x);
      double delta_angle = angle_query - angle_start;
      constexpr double kTwoPi = 2.0 * 3.14159265358979323846;
      if (curvature > 0.0) {
        while (delta_angle < 0.0) {
          delta_angle += kTwoPi;
        }
        while (delta_angle >= kTwoPi) {
          delta_angle -= kTwoPi;
        }
      } else {
        while (delta_angle > 0.0) {
          delta_angle -= kTwoPi;
        }
        while (delta_angle <= -kTwoPi) {
          delta_angle += kTwoPi;
        }
      }
      double s_local = std::clamp(delta_angle / curvature, 0.0, seg_length);
      return s_start + s_local;
    }
  }

  // Fallback to numerical solver for spirals and ParamPoly3
  constexpr int kNumIntervals = 10;
  double best_s = 0.0;
  double min_dist_sq = std::numeric_limits<double>::max();

  for (int i = 0; i <= kNumIntervals; ++i) {
    double s_test = (static_cast<double>(i) / kNumIntervals) * seg_length;
    auto pt = Evaluate(seg_idx, s_start + s_test);
    double dx = px - pt.x;
    double dy = py - pt.y;
    double dist_sq = (dx * dx) + (dy * dy);
    if (dist_sq < min_dist_sq) {
      min_dist_sq = dist_sq;
      best_s = s_test;
    }
  }

  double left_s = std::max(0.0, best_s - (seg_length / kNumIntervals));
  double right_s = std::min(seg_length, best_s + (seg_length / kNumIntervals));
  for (int iter = 0; iter < 30; ++iter) {
    double m1 = left_s + ((right_s - left_s) / 3.0);
    double m2 = right_s - ((right_s - left_s) / 3.0);
    auto pt1 = Evaluate(seg_idx, s_start + m1);
    auto pt2 = Evaluate(seg_idx, s_start + m2);
    double dist1 = ((px - pt1.x) * (px - pt1.x)) + ((py - pt1.y) * (py - pt1.y));
    double dist2 = ((px - pt2.x) * (px - pt2.x)) + ((py - pt2.y) * (py - pt2.y));
    if (dist1 < dist2) {
      right_s = m2;
    } else {
      left_s = m1;
    }
  }

  return s_start + 0.5 * (left_s + right_s);
}

auto ReferenceLine::FindSegmentIndex(RoadId road, double s_coord, QueryContext& ctx) const noexcept -> uint32_t {
  auto road_idx = static_cast<uint32_t>(road);
  uint32_t first_seg = road_ref_line_first_idx_[road_idx];
  uint32_t seg_count = road_ref_line_count_[road_idx];

  uint32_t seg_idx = first_seg;
  bool hit = false;

  if (ctx.last_road == road && ctx.last_segment_idx.has_value()) {
    uint32_t last_idx = *ctx.last_segment_idx;
    if (last_idx >= first_seg && last_idx < first_seg + seg_count) {
      double s_start = s_offset_[last_idx];
      double s_end = s_start + length_[last_idx];
      if (s_coord >= s_start && s_coord < s_end) [[likely]] {
        seg_idx = last_idx;
        hit = true;
      }
    }
  }

  if (!hit) [[unlikely]] {
    for (uint32_t i = 0; i < seg_count; ++i) {
      uint32_t idx = first_seg + i;
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

auto ReferenceLine::GetRoadSegments(RoadId road) const noexcept -> std::pair<uint32_t, uint32_t> {
  auto road_idx = static_cast<uint32_t>(road);
  if (road_idx >= road_ref_line_first_idx_.size()) {
    return {0, 0};
  }
  return {road_ref_line_first_idx_[road_idx], road_ref_line_count_[road_idx]};
}

auto ReferenceLine::GetSegmentSStart(uint32_t seg_idx) const noexcept -> double { return s_offset_[seg_idx]; }

auto ReferenceLine::GetSegmentLength(uint32_t seg_idx) const noexcept -> double { return length_[seg_idx]; }

auto ReferenceLine::ComputeSegmentAabb(uint32_t seg_idx, double inflation) const noexcept -> Aabb {
  double min_x = std::numeric_limits<double>::max();
  double min_y = std::numeric_limits<double>::max();
  double max_x = -std::numeric_limits<double>::max();
  double max_y = -std::numeric_limits<double>::max();

  double length = length_[seg_idx];
  double s_start = s_offset_[seg_idx];

  int num_samples = 1;
  if (type_[seg_idx] != GeometryType::kLine) {
    num_samples = 32;
  }

  for (int idx = 0; idx <= num_samples; ++idx) {
    double s_local = (static_cast<double>(idx) / num_samples) * length;
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
