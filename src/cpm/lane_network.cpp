// SPDX-License-Identifier: BSL-1.0

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <strada/ast/abstract_syntax_tree.hpp>
#include <strada/cpm/coordinate.hpp>
#include <strada/cpm/elevation_profile.hpp>
#include <strada/cpm/ids.hpp>
#include <strada/cpm/lane_network.hpp>
#include <strada/cpm/query_context.hpp>
#include <strada/parser/conversions.hpp>
#include <vector>

namespace strada::cpm {

namespace {

auto EvaluateStripOwnHeight(const Polynomials& poly, const StripsSoA& strips, std::uint32_t strip_idx, double s_coord,
                            double dt_val) noexcept -> double {
  const double kCoeff0 = poly.Evaluate(strips.c0_first_idx[strip_idx], strips.c0_count[strip_idx], s_coord);
  const double kCoeff1 = poly.Evaluate(strips.c1_first_idx[strip_idx], strips.c1_count[strip_idx], s_coord);
  const double kCoeff2 = poly.Evaluate(strips.c2_first_idx[strip_idx], strips.c2_count[strip_idx], s_coord);
  const double kCoeff3 = poly.Evaluate(strips.c3_first_idx[strip_idx], strips.c3_count[strip_idx], s_coord);
  return kCoeff0 + (kCoeff1 * dt_val) + (kCoeff2 * dt_val * dt_val) + (kCoeff3 * dt_val * dt_val * dt_val);
}

auto EvaluateStripHeight(const Polynomials& poly, const StripsSoA& strips, std::uint32_t strip_idx,
                         std::uint32_t first_strip_idx, std::uint32_t strip_count, double s_coord,
                         double dt_val) noexcept -> double {
  double h_accum = EvaluateStripOwnHeight(poly, strips, strip_idx, s_coord, dt_val);
  std::uint32_t curr_strip_idx = strip_idx;

  while (strips.is_relative[curr_strip_idx] != 0U) {
    const std::int32_t kIdVal = strips.strip_id[curr_strip_idx];
    const std::int32_t kInnerId = (kIdVal > 0) ? (kIdVal - 1) : (kIdVal + 1);

    bool found = false;
    for (std::uint32_t j = 0; j < strip_count; ++j) {
      const std::uint32_t kInnerIdx = first_strip_idx + j;
      if (strips.strip_id[kInnerIdx] == kInnerId) {
        const double kInnerW = poly.Evaluate(strips.width_first_idx[kInnerIdx], strips.width_count[kInnerIdx], s_coord);
        const double kInnerDt = (kInnerId > 0) ? kInnerW : -kInnerW;
        h_accum += EvaluateStripOwnHeight(poly, strips, kInnerIdx, s_coord, kInnerDt);
        curr_strip_idx = kInnerIdx;
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

void EvaluateCrossSectionSurfaceOffset(const Polynomials& polynomials, const StripsSoA& strips,
                                       const RoadCrossSectionSurfaceSoA& road_css, std::uint32_t road_idx,
                                       double s_coord, double t_coord, double& h_surf) noexcept {
  h_surf = 0.0;
  const std::uint32_t kCssStripCount = road_css.strip_count.empty() ? 0 : road_css.strip_count[road_idx];
  if (kCssStripCount > 0) {
    const double kTOffset =
        polynomials.Evaluate(road_css.t_offset_first_idx[road_idx], road_css.t_offset_count[road_idx], s_coord);

    const double kTSurf = t_coord - kTOffset;
    const bool kIsLeft = (kTSurf >= 0.0);
    const double kTTarget = kIsLeft ? kTSurf : std::abs(kTSurf);

    const std::uint32_t kFirstStripIdx = road_css.first_strip_idx[road_idx];
    double t_accum = 0.0;

    for (std::uint32_t i = 0; i < kCssStripCount; ++i) {
      const std::uint32_t kStripIdx = kFirstStripIdx + i;
      const std::int32_t kIdVal = strips.strip_id[kStripIdx];
      const bool kStripIsLeft = (kIdVal > 0);

      if (kStripIsLeft == kIsLeft) {
        double width_val = std::numeric_limits<double>::infinity();
        const std::uint32_t kWCount = strips.width_count[kStripIdx];
        if (kWCount > 0) {
          width_val = polynomials.Evaluate(strips.width_first_idx[kStripIdx], kWCount, s_coord);
        }

        if (kTTarget >= t_accum && kTTarget < t_accum + width_val) {
          const double kTStrip = kTTarget - t_accum;
          const double kDtVal = kStripIsLeft ? kTStrip : -kTStrip;

          h_surf = EvaluateStripHeight(polynomials, strips, kStripIdx, kFirstStripIdx, kCssStripCount, s_coord, kDtVal);
          break;
        }
        t_accum += width_val;
      }
    }
  }
}

}  // namespace

auto LaneNetwork::Build(const ast::AbstractSyntaxTree& map) -> LaneNetwork {
  LaneNetwork network;

  for (const auto& road : map.roads) {
    const auto& css_opt = road.lateral_profile.cross_section_surface;
    if (css_opt.has_value()) {
      const auto& css = *css_opt;

      network.road_css_.first_strip_idx.push_back(static_cast<std::uint32_t>(network.strips_.strip_id.size()));
      network.road_css_.strip_count.push_back(static_cast<std::uint32_t>(css.strips.size()));

      const auto [t_off_idx, t_off_cnt] = network.polynomials_.Compile(css.t_offset);
      network.road_css_.t_offset_first_idx.push_back(t_off_idx);
      network.road_css_.t_offset_count.push_back(t_off_cnt);

      auto sorted_strips = css.strips;
      std::ranges::sort(sorted_strips, [](const auto& val_a, const auto& val_b) noexcept -> bool {
        return std::abs(val_a.id) < std::abs(val_b.id);
      });

      for (const auto& strip : sorted_strips) {
        network.strips_.strip_id.push_back(strip.id);
        network.strips_.is_relative.push_back(static_cast<std::uint8_t>(parser::ToString(strip.mode) == "relative"));

        const auto [w_first, w_count] = network.polynomials_.Compile(strip.width);
        network.strips_.width_first_idx.push_back(w_first);
        network.strips_.width_count.push_back(w_count);

        const auto [c0_first, c0_count] = network.polynomials_.Compile(strip.constant);
        network.strips_.c0_first_idx.push_back(c0_first);
        network.strips_.c0_count.push_back(c0_count);

        const auto [c1_first, c1_count] = network.polynomials_.Compile(strip.linear);
        network.strips_.c1_first_idx.push_back(c1_first);
        network.strips_.c1_count.push_back(c1_count);

        const auto [c2_first, c2_count] = network.polynomials_.Compile(strip.quadratic);
        network.strips_.c2_first_idx.push_back(c2_first);
        network.strips_.c2_count.push_back(c2_count);

        const auto [c3_first, c3_count] = network.polynomials_.Compile(strip.cubic);
        network.strips_.c3_first_idx.push_back(c3_first);
        network.strips_.c3_count.push_back(c3_count);
      }
    } else {
      network.road_css_.first_strip_idx.push_back(0);
      network.road_css_.strip_count.push_back(0);
      network.road_css_.t_offset_first_idx.push_back(0);
      network.road_css_.t_offset_count.push_back(0);
    }

    // Lane offset profile compilation (road level)
    {
      const auto first_idx = static_cast<std::uint32_t>(network.lane_offsets_.lane_offset_s_start.size());
      const auto count = static_cast<std::uint32_t>(road.lanes.offsets.size());
      for (const auto& offset : road.lanes.offsets) {
        network.lane_offsets_.lane_offset_s_start.push_back(offset.s);
        network.lane_offsets_.lane_offset_a.push_back(offset.a);
        network.lane_offsets_.lane_offset_b.push_back(offset.b);
        network.lane_offsets_.lane_offset_c.push_back(offset.c);
        network.lane_offsets_.lane_offset_d.push_back(offset.d);
      }
      network.lane_offsets_.road_lane_offset_first_idx.push_back(first_idx);
      network.lane_offsets_.road_lane_offset_count.push_back(count);
    }

    // Lane sections compilation
    {
      const auto road_sec_first = static_cast<std::uint32_t>(network.lane_sections_.section_s.size());
      const auto road_sec_count = static_cast<std::uint32_t>(road.lanes.sections.size());

      for (const auto& section : road.lanes.sections) {
        network.lane_sections_.section_s.push_back(section.s);

        // Accumulate all lanes in this section
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

        const auto section_first_lane = static_cast<std::uint32_t>(network.lanes_.lane_original_id.size());
        const auto section_lane_count = static_cast<std::uint32_t>(sorted_section_lanes.size());

        network.lane_sections_.section_first_lane_idx.push_back(section_first_lane);
        network.lane_sections_.section_lane_count.push_back(section_lane_count);

        // Now compile each lane in this section
        for (const auto& lane : sorted_section_lanes) {
          network.lanes_.lane_original_id.push_back(lane.id);
          network.lanes_.lane_road_id.push_back(static_cast<RoadId>(network.road_css_.strip_count.size() - 1));
          network.lanes_.lane_section_idx.push_back(
              static_cast<std::uint32_t>(network.lane_sections_.section_s.size() - 1));

          // Width polynomials for this lane
          const auto w_first = static_cast<std::uint32_t>(network.lane_widths_.lane_width_s_start.size());
          const auto w_count = static_cast<std::uint32_t>(lane.widths.size());
          for (const auto& width_poly : lane.widths) {
            network.lane_widths_.lane_width_s_start.push_back(section.s + width_poly.s_offset);
            network.lane_widths_.lane_width_a.push_back(width_poly.a);
            network.lane_widths_.lane_width_b.push_back(width_poly.b);
            network.lane_widths_.lane_width_c.push_back(width_poly.c);
            network.lane_widths_.lane_width_d.push_back(width_poly.d);
          }
          network.lanes_.lane_first_width_idx.push_back(w_first);
          network.lanes_.lane_width_count.push_back(w_count);

          // Height polynomials for this lane
          const auto h_first = static_cast<std::uint32_t>(network.lane_heights_.lane_height_s_start.size());
          const auto h_count = static_cast<std::uint32_t>(lane.heights.size());
          for (const auto& height_poly : lane.heights) {
            network.lane_heights_.lane_height_s_start.push_back(section.s + height_poly.s_offset);
            network.lane_heights_.lane_height_inner.push_back(height_poly.inner);
            network.lane_heights_.lane_height_outer.push_back(height_poly.outer);
          }
          network.lanes_.lane_first_height_idx.push_back(h_first);
          network.lanes_.lane_height_count.push_back(h_count);
        }
      }
      network.lane_sections_.road_section_first_idx.push_back(road_sec_first);
      network.lane_sections_.road_section_count.push_back(road_sec_count);
    }
  }

  return network;
}

auto LaneNetwork::RoadToLane(RoadPose pose, QueryContext& ctx) const noexcept -> std::optional<LanePose> {
  const auto road_idx = static_cast<std::uint32_t>(pose.road);
  if (road_idx >= road_css_.strip_count.size()) {
    return std::nullopt;
  }

  const auto road_sec_first = lane_sections_.road_section_first_idx[road_idx];
  const auto road_sec_count = lane_sections_.road_section_count[road_idx];
  if (road_sec_count == 0) {
    return std::nullopt;
  }

  // Find the active lane section at pose.s
  auto sec_idx = road_sec_first;
  for (std::uint32_t i = 0; i < road_sec_count; ++i) {
    auto cur_sec = road_sec_first + i;
    if (pose.s >= lane_sections_.section_s[cur_sec]) {
      sec_idx = cur_sec;
    } else {
      break;
    }
  }

  const auto first_lane_in_sec = lane_sections_.section_first_lane_idx[sec_idx];
  const auto lane_cnt_in_sec = lane_sections_.section_lane_count[sec_idx];

  // Calculate road-level lane offset
  double lane_offset_val = 0.0;
  const auto lo_first = lane_offsets_.road_lane_offset_first_idx[road_idx];
  const auto lo_count = lane_offsets_.road_lane_offset_count[road_idx];
  if (lo_count > 0) {
    auto active_lo = lo_first;
    for (std::uint32_t i = 0; i < lo_count; ++i) {
      auto cur_lo = lo_first + i;
      if (pose.s >= lane_offsets_.lane_offset_s_start[cur_lo]) {
        active_lo = cur_lo;
      } else {
        break;
      }
    }
    const double kDsLo = pose.s - lane_offsets_.lane_offset_s_start[active_lo];
    lane_offset_val =
        lane_offsets_.lane_offset_a[active_lo] +
        (kDsLo * (lane_offsets_.lane_offset_b[active_lo] +
                  kDsLo * (lane_offsets_.lane_offset_c[active_lo] + kDsLo * lane_offsets_.lane_offset_d[active_lo])));
  }

  const double kTRelative = pose.t - lane_offset_val;

  std::uint32_t matched_lane_idx = 0;
  bool found = false;
  double t_center = 0.0;
  double w_target = 0.0;
  int target_id = 0;

  if (kTRelative > 0.0) {
    // Left lanes: IDs > 0, sorted ascending (e.g. 1, 2, 3...)
    double t_inner = 0.0;
    for (std::uint32_t i = 0; i < lane_cnt_in_sec; ++i) {
      const std::uint32_t kLaneIdx = first_lane_in_sec + i;
      const int kLaneId = lanes_.lane_original_id[kLaneIdx];
      if (kLaneId <= 0) {
        continue;
      }
      const double kW = LaneWidth(static_cast<LaneId>(kLaneIdx), pose.s);
      const double kTOuter = t_inner + kW;
      if (kW > 0.0 && kTRelative >= t_inner && kTRelative <= kTOuter) {
        matched_lane_idx = kLaneIdx;
        t_center = t_inner + (0.5 * kW);
        w_target = kW;
        target_id = kLaneId;
        found = true;
        break;
      }
      t_inner = kTOuter;
    }
  } else if (kTRelative < 0.0) {
    // Right lanes: IDs < 0, sorted ascending (e.g. -3, -2, -1)
    // Walk them in reverse order (from -1 down to -3) to go from inside to outside.
    double t_inner = 0.0;
    for (int i = static_cast<int>(lane_cnt_in_sec) - 1; i >= 0; --i) {
      const std::uint32_t kLaneIdx = first_lane_in_sec + static_cast<std::uint32_t>(i);
      const int kLaneId = lanes_.lane_original_id[kLaneIdx];
      if (kLaneId >= 0) {
        continue;
      }
      const double kW = LaneWidth(static_cast<LaneId>(kLaneIdx), pose.s);
      const double kTOuter = t_inner - kW;
      if (kW > 0.0 && kTRelative <= t_inner && kTRelative >= kTOuter) {
        matched_lane_idx = kLaneIdx;
        t_center = t_inner - (0.5 * kW);
        w_target = kW;
        target_id = kLaneId;
        found = true;
        break;
      }
      t_inner = kTOuter;
    }
  }

  if (!found) {
    return std::nullopt;
  }

  // Evaluate lane height offset
  double h_inner = 0.0;
  double h_outer = 0.0;
  const std::uint32_t kHFirst = lanes_.lane_first_height_idx[matched_lane_idx];
  const std::uint32_t kHCount = lanes_.lane_height_count[matched_lane_idx];
  if (kHCount > 0) {
    std::uint32_t active_h = kHFirst;
    for (std::uint32_t i = 0; i < kHCount; ++i) {
      const std::uint32_t kCurH = kHFirst + i;
      if (pose.s >= lane_heights_.lane_height_s_start[kCurH]) {
        active_h = kCurH;
      } else {
        break;
      }
    }
    h_inner = lane_heights_.lane_height_inner[active_h];
    h_outer = lane_heights_.lane_height_outer[active_h];
  }

  const double kTLane = kTRelative - t_center;
  double f = 0.0;
  if (w_target > 0.0) {
    if (target_id > 0) {
      f = 0.5 + (kTLane / w_target);
    } else if (target_id < 0) {
      f = 0.5 - (kTLane / w_target);
    }
  }
  f = std::clamp(f, 0.0, 1.0);
  const double kHOffset = h_inner + (f * (h_outer - h_inner));

  LanePose lane_pose;
  lane_pose.s = pose.s;
  lane_pose.t = kTLane;
  lane_pose.h = pose.h - kHOffset;
  lane_pose.heading = pose.heading;
  lane_pose.pitch = pose.pitch;
  lane_pose.roll = pose.roll;
  lane_pose.road = pose.road;
  lane_pose.lane = static_cast<LaneId>(matched_lane_idx);

  // Update query context road cache
  ctx.last_road = pose.road;

  return lane_pose;
}

auto LaneNetwork::LaneToRoad(LanePose pose, QueryContext& /*ctx*/) const noexcept -> RoadPose {
  const auto lane_idx = static_cast<std::uint32_t>(pose.lane);
  if (lane_idx >= lanes_.lane_original_id.size()) {
    return RoadPose{};
  }

  const double kS = pose.s;
  const int kTargetId = lanes_.lane_original_id[lane_idx];
  const RoadId kRoadId = lanes_.lane_road_id[lane_idx];
  auto road_idx = static_cast<std::uint32_t>(kRoadId);
  const std::uint32_t kSecIdx = lanes_.lane_section_idx[lane_idx];

  // 1. Compute cumulative inner boundary width
  double inner_boundary_t = 0.0;
  const std::uint32_t kFirstLaneInSec = lane_sections_.section_first_lane_idx[kSecIdx];
  const std::uint32_t kLaneCntInSec = lane_sections_.section_lane_count[kSecIdx];

  for (std::uint32_t i = 0; i < kLaneCntInSec; ++i) {
    const std::uint32_t kOtherIdx = kFirstLaneInSec + i;
    const int kOtherId = lanes_.lane_original_id[kOtherIdx];
    if (kTargetId > 0) {
      if (kOtherId > 0 && kOtherId < kTargetId) {
        inner_boundary_t += LaneWidth(static_cast<LaneId>(kOtherIdx), kS);
      }
    } else if (kTargetId < 0) {
      if (kOtherId < 0 && kOtherId > kTargetId) {
        inner_boundary_t += LaneWidth(static_cast<LaneId>(kOtherIdx), kS);
      }
    }
  }

  if (kTargetId < 0) {
    inner_boundary_t = -inner_boundary_t;
  }

  // 2. Target lane width
  const double kWTarget = LaneWidth(pose.lane, kS);

  // 3. Center line t of the lane
  double t_center = 0.0;
  if (kTargetId > 0) {
    t_center = inner_boundary_t + (0.5 * kWTarget);
  } else if (kTargetId < 0) {
    t_center = inner_boundary_t - (0.5 * kWTarget);
  }

  double road_t = t_center + pose.t;

  // 4. Add road-level laneOffset
  double lane_offset_val = 0.0;
  const std::uint32_t kLoFirst = lane_offsets_.road_lane_offset_first_idx[road_idx];
  const std::uint32_t kLoCount = lane_offsets_.road_lane_offset_count[road_idx];
  if (kLoCount > 0) {
    std::uint32_t active_lo = kLoFirst;
    for (std::uint32_t i = 0; i < kLoCount; ++i) {
      const std::uint32_t kCurLo = kLoFirst + i;
      if (kS >= lane_offsets_.lane_offset_s_start[kCurLo]) {
        active_lo = kCurLo;
      } else {
        break;
      }
    }
    const double kDsLo = kS - lane_offsets_.lane_offset_s_start[active_lo];
    lane_offset_val = lane_offsets_.lane_offset_a[active_lo] + (lane_offsets_.lane_offset_b[active_lo] * kDsLo) +
                      (lane_offsets_.lane_offset_c[active_lo] * kDsLo * kDsLo) +
                      (lane_offsets_.lane_offset_d[active_lo] * kDsLo * kDsLo * kDsLo);
  }
  road_t += lane_offset_val;

  // 5. Evaluate lane height offset
  double h_inner = 0.0;
  double h_outer = 0.0;
  const std::uint32_t kHFirst = lanes_.lane_first_height_idx[lane_idx];
  const std::uint32_t kHCount = lanes_.lane_height_count[lane_idx];
  if (kHCount > 0) {
    std::uint32_t active_h = kHFirst;
    for (std::uint32_t i = 0; i < kHCount; ++i) {
      const std::uint32_t kCurH = kHFirst + i;
      if (kS >= lane_heights_.lane_height_s_start[kCurH]) {
        active_h = kCurH;
      } else {
        break;
      }
    }
    h_inner = lane_heights_.lane_height_inner[active_h];
    h_outer = lane_heights_.lane_height_outer[active_h];
  }

  double f = 0.0;
  if (kWTarget > 0.0) {
    if (kTargetId > 0) {
      f = 0.5 + (pose.t / kWTarget);
    } else if (kTargetId < 0) {
      f = 0.5 - (pose.t / kWTarget);
    }
  }
  f = std::clamp(f, 0.0, 1.0);
  const double kHOffset = h_inner + (f * (h_outer - h_inner));
  const double kRoadH = pose.h + kHOffset;

  RoadPose road_pose;
  road_pose.s = kS;
  road_pose.t = road_t;
  road_pose.h = kRoadH;
  road_pose.heading = pose.heading;
  road_pose.pitch = pose.pitch;
  road_pose.roll = pose.roll;
  road_pose.road = kRoadId;
  return road_pose;
}

auto LaneNetwork::LaneCount() const noexcept -> std::size_t { return lanes_.lane_original_id.size(); }

auto LaneNetwork::LaneRoad(LaneId lane_id) const noexcept -> RoadId {
  auto idx = static_cast<std::uint32_t>(lane_id);
  if (idx < lanes_.lane_road_id.size()) {
    return lanes_.lane_road_id[idx];
  }
  return RoadId{0};
}

auto LaneNetwork::OriginalLaneId(LaneId lane_id) const noexcept -> int {
  auto idx = static_cast<std::uint32_t>(lane_id);
  if (idx < lanes_.lane_original_id.size()) {
    return lanes_.lane_original_id[idx];
  }
  return 0;
}

auto LaneNetwork::LaneWidth(LaneId lane_id, double s_coord) const noexcept -> double {
  auto idx = static_cast<std::uint32_t>(lane_id);
  if (idx >= lanes_.lane_original_id.size()) {
    return 0.0;
  }
  const std::uint32_t kWFirst = lanes_.lane_first_width_idx[idx];
  const std::uint32_t kWCount = lanes_.lane_width_count[idx];
  if (kWCount == 0) {
    return 0.0;
  }

  std::uint32_t active_idx = kWFirst;
  for (std::uint32_t i = 0; i < kWCount; ++i) {
    const std::uint32_t kCurIdx = kWFirst + i;
    if (s_coord >= lane_widths_.lane_width_s_start[kCurIdx]) {
      active_idx = kCurIdx;
    } else {
      break;
    }
  }

  const double kDs = s_coord - lane_widths_.lane_width_s_start[active_idx];
  return lane_widths_.lane_width_a[active_idx] + (lane_widths_.lane_width_b[active_idx] * kDs) +
         (lane_widths_.lane_width_c[active_idx] * kDs * kDs) +
         (lane_widths_.lane_width_d[active_idx] * kDs * kDs * kDs);
}

auto LaneNetwork::EvaluateCrossSectionSurfaceOffset(RoadId road, double s_coord, double t_coord) const noexcept
    -> double {
  double h_surf = 0.0;
  auto road_idx = static_cast<std::uint32_t>(road);
  if (road_idx >= road_css_.strip_count.size()) {
    return 0.0;
  }
  strada::cpm::EvaluateCrossSectionSurfaceOffset(polynomials_, strips_, road_css_, road_idx, s_coord, t_coord, h_surf);
  return h_surf;
}

void LaneNetwork::GetRoadWidthLimits(RoadId road, double s_coord, double& t_left, double& t_right) const noexcept {
  t_left = 0.0;
  t_right = 0.0;

  auto road_idx = static_cast<std::uint32_t>(road);
  if (road_idx >= lane_sections_.road_section_first_idx.size()) {
    return;
  }

  auto road_sec_first = lane_sections_.road_section_first_idx[road_idx];
  auto road_sec_count = lane_sections_.road_section_count[road_idx];
  if (road_sec_count == 0) {
    return;
  }

  auto sec_idx = road_sec_first;
  for (std::uint32_t i = 0; i < road_sec_count; ++i) {
    auto cur_sec = road_sec_first + i;
    if (s_coord >= lane_sections_.section_s[cur_sec]) {
      sec_idx = cur_sec;
    } else {
      break;
    }
  }

  auto first_lane_in_sec = lane_sections_.section_first_lane_idx[sec_idx];
  auto lane_cnt_in_sec = lane_sections_.section_lane_count[sec_idx];

  for (std::uint32_t i = 0; i < lane_cnt_in_sec; ++i) {
    auto lane_idx = first_lane_in_sec + i;
    auto lane_id = lanes_.lane_original_id[lane_idx];
    const double kW = LaneWidth(static_cast<LaneId>(lane_idx), s_coord);
    if (lane_id > 0) {
      t_left += kW;
    } else if (lane_id < 0) {
      t_right -= kW;
    }
  }

  double lane_offset_val = 0.0;
  auto lo_first = lane_offsets_.road_lane_offset_first_idx[road_idx];
  auto lo_count = lane_offsets_.road_lane_offset_count[road_idx];
  if (lo_count > 0) {
    auto active_lo = lo_first;
    for (std::uint32_t i = 0; i < lo_count; ++i) {
      auto cur_lo = lo_first + i;
      if (s_coord >= lane_offsets_.lane_offset_s_start[cur_lo]) {
        active_lo = cur_lo;
      } else {
        break;
      }
    }
    const double kDsLo = s_coord - lane_offsets_.lane_offset_s_start[active_lo];
    lane_offset_val =
        lane_offsets_.lane_offset_a[active_lo] +
        (kDsLo * (lane_offsets_.lane_offset_b[active_lo] +
                  kDsLo * (lane_offsets_.lane_offset_c[active_lo] + kDsLo * lane_offsets_.lane_offset_d[active_lo])));
  }

  t_left += lane_offset_val;
  t_right += lane_offset_val;
}

auto LaneNetwork::FindLaneId(RoadId road_id, std::uint32_t relative_section_idx, int original_lane_id) const noexcept
    -> std::optional<LaneId> {
  const auto kRoadIdx = static_cast<std::uint32_t>(road_id);
  if (kRoadIdx >= lane_sections_.road_section_first_idx.size()) {
    return std::nullopt;
  }

  const auto kRoadSecFirst = lane_sections_.road_section_first_idx[kRoadIdx];
  const auto kRoadSecCount = lane_sections_.road_section_count[kRoadIdx];
  if (relative_section_idx >= kRoadSecCount) {
    return std::nullopt;
  }

  const std::uint32_t kAbsSecIdx = kRoadSecFirst + relative_section_idx;
  if (kAbsSecIdx >= lane_sections_.section_first_lane_idx.size()) {
    return std::nullopt;
  }

  const std::uint32_t kFirstLaneIdx = lane_sections_.section_first_lane_idx[kAbsSecIdx];
  const std::uint32_t kLaneCount = lane_sections_.section_lane_count[kAbsSecIdx];

  for (std::uint32_t i = 0; i < kLaneCount; ++i) {
    const std::uint32_t kLaneIdx = kFirstLaneIdx + i;
    if (kLaneIdx < lanes_.lane_original_id.size()) {
      if (lanes_.lane_original_id[kLaneIdx] == original_lane_id) {
        return static_cast<LaneId>(kLaneIdx);
      }
    }
  }

  return std::nullopt;
}

auto LaneNetwork::GetMaxRoadWidth(RoadId road_id, double road_length) const noexcept -> double {
  double max_road_t = 0.0;
  const auto kRoadIdx = static_cast<std::uint32_t>(road_id);
  if (kRoadIdx >= lane_sections_.road_section_first_idx.size()) {
    return 0.0;
  }

  const auto kRoadSecFirst = lane_sections_.road_section_first_idx[kRoadIdx];
  const auto kRoadSecCount = lane_sections_.road_section_count[kRoadIdx];

  for (std::uint32_t sec_idx = 0; sec_idx < kRoadSecCount; ++sec_idx) {
    const std::uint32_t kCurSec = kRoadSecFirst + sec_idx;
    const double kSecSStart = lane_sections_.section_s[kCurSec];
    double sec_length = 0.0;
    if (sec_idx + 1 < kRoadSecCount) {
      sec_length = lane_sections_.section_s[kCurSec + 1] - kSecSStart;
    } else {
      sec_length = road_length - kSecSStart;
    }

    constexpr int kSecSamples = 10;
    for (int i = 0; i <= kSecSamples; ++i) {
      const double kSLocal = (static_cast<double>(i) / kSecSamples) * sec_length;
      const double kS = kSecSStart + kSLocal;
      double t_left = 0.0;
      double t_right = 0.0;
      GetRoadWidthLimits(road_id, kS, t_left, t_right);

      max_road_t = std::max(max_road_t, std::abs(t_left));
      max_road_t = std::max(max_road_t, std::abs(t_right));
    }
  }

  constexpr double kRoadWidthSafetyBuffer = 0.1;
  return max_road_t + kRoadWidthSafetyBuffer;
}

}  // namespace strada::cpm
