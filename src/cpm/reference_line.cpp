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

auto ReferenceLine::Build(const ast::AbstractSyntaxTree& map) -> ReferenceLine {
  ReferenceLine rl;

  for (const auto& road : map.roads) {
    rl.road_ref_line_first_idx_.push_back(static_cast<std::uint32_t>(rl.s_offset_.size()));
    rl.road_ref_line_count_.push_back(static_cast<std::uint32_t>(road.plan_view.size()));

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
        rl.type_index_.push_back(static_cast<std::uint32_t>(rl.arc_curvature_.size()));
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
        const ast::ParamPoly3 kParam = ConvertPoly3ToParamPoly3(geom.length, poly.a, poly.b, poly.c, poly.d);
        rl.pp3_a_u_.push_back(kParam.a_u);
        rl.pp3_b_u_.push_back(kParam.b_u);
        rl.pp3_c_u_.push_back(kParam.c_u);
        rl.pp3_d_u_.push_back(kParam.d_u);
        rl.pp3_a_v_.push_back(kParam.a_v);
        rl.pp3_b_v_.push_back(kParam.b_v);
        rl.pp3_c_v_.push_back(kParam.c_v);
        rl.pp3_d_v_.push_back(kParam.d_v);
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

auto ReferenceLine::Evaluate(std::uint32_t seg_idx, double road_s) const noexcept -> ReferenceLinePoint {
  const double kDsVal = road_s - s_offset_[seg_idx];
  double ref_x = x_[seg_idx];
  double ref_y = y_[seg_idx];
  const double kRefHdg = hdg_[seg_idx];
  double tangent_hdg = kRefHdg;

  const GeometryType kType = type_[seg_idx];
  const std::uint32_t kTypeIdx = type_index_[seg_idx];

  if (kType == GeometryType::kLine) {
    ref_x += kDsVal * std::cos(kRefHdg);
    ref_y += kDsVal * std::sin(kRefHdg);
  } else if (kType == GeometryType::kArc) {
    const double kCurvature = arc_curvature_[kTypeIdx];
    if (std::abs(kCurvature) > kCurvatureThreshold) {
      tangent_hdg += kCurvature * kDsVal;
      ref_x += (1.0 / kCurvature) * (std::sin(tangent_hdg) - std::sin(kRefHdg));
      ref_y -= (1.0 / kCurvature) * (std::cos(tangent_hdg) - std::cos(kRefHdg));
    } else {
      ref_x += kDsVal * std::cos(kRefHdg);
      ref_y += kDsVal * std::sin(kRefHdg);
    }
  } else if (kType == GeometryType::kSpiral) {
    const double kCurvStart = spiral_curv_start_[seg_idx];
    const double kCurvEnd = spiral_curv_end_[seg_idx];
    const double kLength = length_[seg_idx];
    const double kLambda = (kLength > 0.0) ? ((kCurvEnd - kCurvStart) / kLength) : 0.0;

    tangent_hdg += (kCurvStart * kDsVal) + (0.5 * (kLambda * (kDsVal * kDsVal)));

    const double kParamA = kLambda * kDsVal * kDsVal;
    const double kParamB = kCurvStart * kDsVal;
    auto [local_x, local_y] = EvaluateClothoidIntegrals(kParamA, kParamB);

    ref_x += kDsVal * (std::cos(kRefHdg) * local_x - std::sin(kRefHdg) * local_y);
    ref_y += kDsVal * (std::sin(kRefHdg) * local_x + std::cos(kRefHdg) * local_y);
  } else if (kType == GeometryType::kParamPoly3) {
    const double kAU = pp3_a_u_[seg_idx];
    const double kBU = pp3_b_u_[seg_idx];
    const double kCU = pp3_c_u_[seg_idx];
    const double kDU = pp3_d_u_[seg_idx];
    const double kAV = pp3_a_v_[seg_idx];
    const double kBV = pp3_b_v_[seg_idx];
    const double kCV = pp3_c_v_[seg_idx];
    const double kDV = pp3_d_v_[seg_idx];
    const std::uint8_t kPRange = pp3_p_range_[seg_idx];

    const double kLength = length_[seg_idx];
    double p_val = kDsVal;
    if (kPRange == 0U) {
      p_val = (kLength > 0.0) ? (kDsVal / kLength) : 0.0;
    }

    const double kUP = kAU + (p_val * (kBU + (p_val * (kCU + (kDU * p_val)))));
    const double kVP = kAV + (p_val * (kBV + (p_val * (kCV + (kDV * p_val)))));

    const double kDuDp = kBU + (p_val * ((kPolyCoeff2 * kCU) + (kPolyCoeff3 * kDU * p_val)));
    const double kDvDp = kBV + (p_val * ((kPolyCoeff2 * kCV) + (kPolyCoeff3 * kDV * p_val)));

    const double kCosHdg = std::cos(kRefHdg);
    const double kSinHdg = std::sin(kRefHdg);

    ref_x += (kUP * kCosHdg) - (kVP * kSinHdg);
    ref_y += (kUP * kSinHdg) + (kVP * kCosHdg);
    tangent_hdg += std::atan2(kDvDp, kDuDp);
  }

  return ReferenceLinePoint{.x = ref_x, .y = ref_y, .heading = tangent_hdg};
}

auto ReferenceLine::Project(std::uint32_t seg_idx, double px, double py) const noexcept -> double {
  const double kSStart = s_offset_[seg_idx];
  const double kSegLength = length_[seg_idx];
  const GeometryType kType = type_[seg_idx];
  const std::uint32_t kTypeIdx = type_index_[seg_idx];

  if (kType == GeometryType::kLine) {
    const double kDx = px - x_[seg_idx];
    const double kDy = py - y_[seg_idx];
    const double kHdg = hdg_[seg_idx];
    const double kDs = (kDx * std::cos(kHdg)) + (kDy * std::sin(kHdg));
    const double kSLocal = std::clamp(kDs, 0.0, kSegLength);
    return kSStart + kSLocal;
  }
  if (kType == GeometryType::kArc) {
    const double kDx = px - x_[seg_idx];
    const double kDy = py - y_[seg_idx];
    const double kHdg = hdg_[seg_idx];
    const double kCurvature = arc_curvature_[kTypeIdx];
    if (std::abs(kCurvature) < kCurvatureThreshold) {
      const double kDs = (kDx * std::cos(kHdg)) + (kDy * std::sin(kHdg));
      const double kSLocal = std::clamp(kDs, 0.0, kSegLength);
      return kSStart + kSLocal;
    }
    const double kRadius = 1.0 / kCurvature;
    const double kCenterX = x_[seg_idx] - (kRadius * std::sin(kHdg));
    const double kCenterY = y_[seg_idx] + (kRadius * std::cos(kHdg));
    const double kQdx = px - kCenterX;
    const double kQdy = py - kCenterY;
    const double kAngleQuery = std::atan2(kQdy, kQdx);
    const double kAngleStart = std::atan2(y_[seg_idx] - kCenterY, x_[seg_idx] - kCenterX);
    double delta_angle = kAngleQuery - kAngleStart;
    constexpr double kTwoPi = 2.0 * std::numbers::pi;
    if (kCurvature > 0.0) {
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
    const double kSLocal = std::clamp(delta_angle / kCurvature, 0.0, kSegLength);
    return kSStart + kSLocal;
  }

  // Fallback to numerical solver for spirals and ParamPoly3
  constexpr int kNumIntervals = 10;
  double best_s = 0.0;
  double min_dist_sq = std::numeric_limits<double>::max();

  for (int i = 0; i <= kNumIntervals; ++i) {
    const double kSTest = (static_cast<double>(i) / kNumIntervals) * kSegLength;
    auto pt = Evaluate(seg_idx, kSStart + kSTest);
    const double kDx = px - pt.x;
    const double kDy = py - pt.y;
    const double kDistSq = (kDx * kDx) + (kDy * kDy);
    if (kDistSq < min_dist_sq) {
      min_dist_sq = kDistSq;
      best_s = kSTest;
    }
  }

  double left_s = std::max(0.0, best_s - (kSegLength / kNumIntervals));
  double right_s = std::min(kSegLength, best_s + (kSegLength / kNumIntervals));
  for (int iter = 0; iter < 30; ++iter) {
    const double kM1 = left_s + ((right_s - left_s) / 3.0);
    const double kM2 = right_s - ((right_s - left_s) / 3.0);
    auto pt1 = Evaluate(seg_idx, kSStart + kM1);
    auto pt2 = Evaluate(seg_idx, kSStart + kM2);
    const double kDist1 = ((px - pt1.x) * (px - pt1.x)) + ((py - pt1.y) * (py - pt1.y));
    const double kDist2 = ((px - pt2.x) * (px - pt2.x)) + ((py - pt2.y) * (py - pt2.y));
    if (kDist1 < kDist2) {
      right_s = kM2;
    } else {
      left_s = kM1;
    }
  }

  return kSStart + (0.5 * (left_s + right_s));
}

auto ReferenceLine::FindSegmentIndex(RoadId road, double s_coord, QueryContext& ctx) const noexcept -> std::uint32_t {
  auto road_idx = static_cast<std::uint32_t>(road);
  const std::uint32_t kFirstSeg = road_ref_line_first_idx_[road_idx];
  const std::uint32_t kSegCount = road_ref_line_count_[road_idx];

  std::uint32_t seg_idx = kFirstSeg;
  bool hit = false;

  if (ctx.last_road == road && ctx.last_segment_idx.has_value()) {
    const std::uint32_t kLastIdx = *ctx.last_segment_idx;
    if (kLastIdx >= kFirstSeg && kLastIdx < kFirstSeg + kSegCount) {
      const double kSStart = s_offset_[kLastIdx];
      const double kSEnd = kSStart + length_[kLastIdx];
      if (s_coord >= kSStart && s_coord < kSEnd) [[likely]] {
        seg_idx = kLastIdx;
        hit = true;
      }
    }
  }

  if (!hit) [[unlikely]] {
    for (std::uint32_t i = 0; i < kSegCount; ++i) {
      const std::uint32_t kIdx = kFirstSeg + i;
      if (s_coord >= s_offset_[kIdx]) {
        seg_idx = kIdx;
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

  const double kLength = length_[seg_idx];
  const double kSStart = s_offset_[seg_idx];

  int num_samples = 1;
  if (type_[seg_idx] != GeometryType::kLine) {
    num_samples = 32;
  }

  for (int idx = 0; idx <= num_samples; ++idx) {
    const double kSLocal = (static_cast<double>(idx) / num_samples) * kLength;
    auto pt = Evaluate(seg_idx, kSStart + kSLocal);
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
