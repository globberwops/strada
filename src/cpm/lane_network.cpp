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
  const double coeff0 = poly.Evaluate(strips.c0_first_idx[strip_idx], strips.c0_count[strip_idx], s_coord);
  const double coeff1 = poly.Evaluate(strips.c1_first_idx[strip_idx], strips.c1_count[strip_idx], s_coord);
  const double coeff2 = poly.Evaluate(strips.c2_first_idx[strip_idx], strips.c2_count[strip_idx], s_coord);
  const double coeff3 = poly.Evaluate(strips.c3_first_idx[strip_idx], strips.c3_count[strip_idx], s_coord);
  return coeff0 + (coeff1 * dt_val) + (coeff2 * dt_val * dt_val) + (coeff3 * dt_val * dt_val * dt_val);
}

auto EvaluateStripHeight(const Polynomials& poly, const StripsSoA& strips, std::uint32_t strip_idx,
                         std::uint32_t first_strip_idx, std::uint32_t strip_count, double s_coord,
                         double dt_val) noexcept -> double {
  double h_accum = EvaluateStripOwnHeight(poly, strips, strip_idx, s_coord, dt_val);
  std::uint32_t curr_strip_idx = strip_idx;

  while (strips.is_relative[curr_strip_idx] != 0U) {
    const std::int32_t id_val = strips.strip_id[curr_strip_idx];
    const std::int32_t inner_id = (id_val > 0) ? (id_val - 1) : (id_val + 1);

    bool found = false;
    for (std::uint32_t j = 0; j < strip_count; ++j) {
      const std::uint32_t inner_idx = first_strip_idx + j;
      if (strips.strip_id[inner_idx] == inner_id) {
        const double inner_w = poly.Evaluate(strips.width_first_idx[inner_idx], strips.width_count[inner_idx], s_coord);
        const double inner_dt = (inner_id > 0) ? inner_w : -inner_w;
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

void EvaluateCrossSectionSurfaceOffset(const Polynomials& polynomials, const StripsSoA& strips,
                                       const RoadCrossSectionSurfaceSoA& road_css, std::uint32_t road_idx,
                                       double s_coord, double t_coord, double& h_surf) noexcept {
  h_surf = 0.0;
  const std::uint32_t css_strip_count = road_css.strip_count.empty() ? 0 : road_css.strip_count[road_idx];
  if (css_strip_count > 0) {
    const double t_offset =
        polynomials.Evaluate(road_css.t_offset_first_idx[road_idx], road_css.t_offset_count[road_idx], s_coord);

    const double t_surf = t_coord - t_offset;
    const bool is_left = (t_surf >= 0.0);
    const double t_target = is_left ? t_surf : std::abs(t_surf);

    const std::uint32_t first_strip_idx = road_css.first_strip_idx[road_idx];
    double t_accum = 0.0;

    for (std::uint32_t i = 0; i < css_strip_count; ++i) {
      const std::uint32_t strip_idx = first_strip_idx + i;
      const std::int32_t id_val = strips.strip_id[strip_idx];
      const bool strip_is_left = (id_val > 0);

      if (strip_is_left == is_left) {
        double width_val = std::numeric_limits<double>::infinity();
        const std::uint32_t w_count = strips.width_count[strip_idx];
        if (w_count > 0) {
          width_val = polynomials.Evaluate(strips.width_first_idx[strip_idx], w_count, s_coord);
        }

        if (t_target >= t_accum && t_target < t_accum + width_val) {
          const double t_strip = t_target - t_accum;
          const double dt_val = strip_is_left ? t_strip : -t_strip;

          h_surf =
              EvaluateStripHeight(polynomials, strips, strip_idx, first_strip_idx, css_strip_count, s_coord, dt_val);
          break;
        }
        t_accum += width_val;
      }
    }
  }
}

void CompileCrossSectionSurface(const ast::Road& road, RoadCrossSectionSurfaceSoA& road_css, StripsSoA& strips,
                                Polynomials& polynomials) {
  const auto& css_opt = road.lateral_profile.cross_section_surface;
  if (css_opt.has_value()) {
    const auto& css = *css_opt;

    road_css.first_strip_idx.push_back(static_cast<std::uint32_t>(strips.strip_id.size()));
    road_css.strip_count.push_back(static_cast<std::uint32_t>(css.strips.size()));

    const auto [t_off_idx, t_off_cnt] = polynomials.Compile(css.t_offset);
    road_css.t_offset_first_idx.push_back(t_off_idx);
    road_css.t_offset_count.push_back(t_off_cnt);

    auto sorted_strips = css.strips;
    std::ranges::sort(sorted_strips, [](const auto& val_a, const auto& val_b) noexcept -> bool {
      return std::abs(val_a.id) < std::abs(val_b.id);
    });

    for (const auto& strip : sorted_strips) {
      strips.strip_id.push_back(strip.id);
      strips.is_relative.push_back(static_cast<std::uint8_t>(parser::ToString(strip.mode) == "relative"));

      const auto [w_first, w_count] = polynomials.Compile(strip.width);
      strips.width_first_idx.push_back(w_first);
      strips.width_count.push_back(w_count);

      const auto [c0_first, c0_count] = polynomials.Compile(strip.constant);
      strips.c0_first_idx.push_back(c0_first);
      strips.c0_count.push_back(c0_count);

      const auto [c1_first, c1_count] = polynomials.Compile(strip.linear);
      strips.c1_first_idx.push_back(c1_first);
      strips.c1_count.push_back(c1_count);

      const auto [c2_first, c2_count] = polynomials.Compile(strip.quadratic);
      strips.c2_first_idx.push_back(c2_first);
      strips.c2_count.push_back(c2_count);

      const auto [c3_first, c3_count] = polynomials.Compile(strip.cubic);
      strips.c3_first_idx.push_back(c3_first);
      strips.c3_count.push_back(c3_count);
    }
  } else {
    road_css.first_strip_idx.push_back(0);
    road_css.strip_count.push_back(0);
    road_css.t_offset_first_idx.push_back(0);
    road_css.t_offset_count.push_back(0);
  }
}

void CompileLaneOffsets(const ast::Road& road, LaneOffsetsSoA& lane_offsets) {
  const auto first_idx = static_cast<std::uint32_t>(lane_offsets.lane_offset_s_start.size());
  const auto count = static_cast<std::uint32_t>(road.lanes.offsets.size());
  for (const auto& offset : road.lanes.offsets) {
    lane_offsets.lane_offset_s_start.push_back(offset.s);
    lane_offsets.lane_offset_a.push_back(offset.a);
    lane_offsets.lane_offset_b.push_back(offset.b);
    lane_offsets.lane_offset_c.push_back(offset.c);
    lane_offsets.lane_offset_d.push_back(offset.d);
  }
  lane_offsets.road_lane_offset_first_idx.push_back(first_idx);
  lane_offsets.road_lane_offset_count.push_back(count);
}

void CompileLaneSections(const ast::Road& road, std::uint32_t road_css_strip_count_size, LaneSectionsSoA& lane_sections,
                         LanesSoA& lanes, LaneWidthsSoA& lane_widths, LaneHeightsSoA& lane_heights) {
  const auto road_sec_first = static_cast<std::uint32_t>(lane_sections.section_s.size());
  const auto road_sec_count = static_cast<std::uint32_t>(road.lanes.sections.size());

  for (const auto& section : road.lanes.sections) {
    lane_sections.section_s.push_back(section.s);

    std::vector<ast::Lane> sorted_section_lanes;
    sorted_section_lanes.reserve(section.right.size() + section.center.size() + section.left.size());
    for (const auto* lane_group : {&section.right, &section.center, &section.left}) {
      for (const auto& lane : *lane_group) {
        sorted_section_lanes.push_back(lane);
      }
    }

    std::ranges::sort(sorted_section_lanes, [](const auto& lhs_lane, const auto& rhs_lane) noexcept -> bool {
      return lhs_lane.id < rhs_lane.id;
    });

    const auto section_first_lane = static_cast<std::uint32_t>(lanes.lane_original_id.size());
    const auto section_lane_count = static_cast<std::uint32_t>(sorted_section_lanes.size());

    lane_sections.section_first_lane_idx.push_back(section_first_lane);
    lane_sections.section_lane_count.push_back(section_lane_count);

    for (const auto& lane : sorted_section_lanes) {
      lanes.lane_original_id.push_back(lane.id);
      lanes.lane_road_id.push_back(static_cast<RoadId>(road_css_strip_count_size - 1));
      lanes.lane_section_idx.push_back(static_cast<std::uint32_t>(lane_sections.section_s.size() - 1));

      const auto w_first = static_cast<std::uint32_t>(lane_widths.lane_width_s_start.size());
      const auto w_count = static_cast<std::uint32_t>(lane.widths.size());
      for (const auto& width_poly : lane.widths) {
        lane_widths.lane_width_s_start.push_back(section.s + width_poly.s_offset);
        lane_widths.lane_width_a.push_back(width_poly.a);
        lane_widths.lane_width_b.push_back(width_poly.b);
        lane_widths.lane_width_c.push_back(width_poly.c);
        lane_widths.lane_width_d.push_back(width_poly.d);
      }
      lanes.lane_first_width_idx.push_back(w_first);
      lanes.lane_width_count.push_back(w_count);

      const auto h_first = static_cast<std::uint32_t>(lane_heights.lane_height_s_start.size());
      const auto h_count = static_cast<std::uint32_t>(lane.heights.size());
      for (const auto& height_poly : lane.heights) {
        lane_heights.lane_height_s_start.push_back(section.s + height_poly.s_offset);
        lane_heights.lane_height_inner.push_back(height_poly.inner);
        lane_heights.lane_height_outer.push_back(height_poly.outer);
      }
      lanes.lane_first_height_idx.push_back(h_first);
      lanes.lane_height_count.push_back(h_count);
    }
  }
  lane_sections.road_section_first_idx.push_back(road_sec_first);
  lane_sections.road_section_count.push_back(road_sec_count);
}

auto EvaluateRoadLaneOffset(const LaneOffsetsSoA& lane_offsets, std::uint32_t road_idx, double s_coord) noexcept
    -> double {
  const std::uint32_t lo_first = lane_offsets.road_lane_offset_first_idx[road_idx];
  const std::uint32_t lo_count = lane_offsets.road_lane_offset_count[road_idx];
  if (lo_count == 0) {
    return 0.0;
  }
  std::uint32_t active_lo = lo_first;
  for (std::uint32_t i = 0; i < lo_count; ++i) {
    const std::uint32_t cur_lo = lo_first + i;
    if (s_coord >= lane_offsets.lane_offset_s_start[cur_lo]) {
      active_lo = cur_lo;
    } else {
      break;
    }
  }
  const double ds_lo = s_coord - lane_offsets.lane_offset_s_start[active_lo];
  return lane_offsets.lane_offset_a[active_lo] +
         (ds_lo * (lane_offsets.lane_offset_b[active_lo] +
                   ds_lo * (lane_offsets.lane_offset_c[active_lo] + ds_lo * lane_offsets.lane_offset_d[active_lo])));
}

auto EvaluateLaneHeightOffset(const LanesSoA& lanes, const LaneHeightsSoA& lane_heights, std::uint32_t lane_idx,
                              double s_coord, double t_lane, int target_id, double w_target) noexcept -> double {
  const std::uint32_t h_first = lanes.lane_first_height_idx[lane_idx];
  const std::uint32_t h_count = lanes.lane_height_count[lane_idx];
  if (h_count == 0) {
    return 0.0;
  }
  std::uint32_t active_h = h_first;
  for (std::uint32_t i = 0; i < h_count; ++i) {
    const std::uint32_t cur_h = h_first + i;
    if (s_coord >= lane_heights.lane_height_s_start[cur_h]) {
      active_h = cur_h;
    } else {
      break;
    }
  }
  const double h_inner = lane_heights.lane_height_inner[active_h];
  const double h_outer = lane_heights.lane_height_outer[active_h];

  double factor = 0.0;
  if (w_target > 0.0) {
    if (target_id > 0) {
      factor = 0.5 + (t_lane / w_target);
    } else if (target_id < 0) {
      factor = 0.5 - (t_lane / w_target);
    }
  }
  factor = std::clamp(factor, 0.0, 1.0);
  return h_inner + (factor * (h_outer - h_inner));
}
auto FindActiveSectionIndex(const LaneSectionsSoA& lane_sections, std::uint32_t road_sec_first,
                            std::uint32_t road_sec_count, double s_coord) noexcept -> std::uint32_t {
  auto sec_idx = road_sec_first;
  for (std::uint32_t i = 0; i < road_sec_count; ++i) {
    const auto cur_sec = road_sec_first + i;
    if (s_coord >= lane_sections.section_s[cur_sec]) {
      sec_idx = cur_sec;
    } else {
      break;
    }
  }
  return sec_idx;
}

}  // namespace

LaneNetwork::LaneNetwork(const ast::AbstractSyntaxTree& map) {
  for (const auto& road : map.roads) {
    CompileCrossSectionSurface(road, road_css_, strips_, polynomials_);
    CompileLaneOffsets(road, lane_offsets_);
    CompileLaneSections(road, static_cast<std::uint32_t>(road_css_.strip_count.size()), lane_sections_, lanes_,
                        lane_widths_, lane_heights_);
  }
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

  const auto sec_idx = FindActiveSectionIndex(lane_sections_, road_sec_first, road_sec_count, pose.s);

  const auto first_lane_in_sec = lane_sections_.section_first_lane_idx[sec_idx];
  const auto lane_cnt_in_sec = lane_sections_.section_lane_count[sec_idx];

  const double lane_offset_val = EvaluateRoadLaneOffset(lane_offsets_, road_idx, pose.s);
  const double t_relative = pose.t - lane_offset_val;

  std::uint32_t matched_lane_idx = 0;
  bool found = false;
  double t_center = 0.0;
  double w_target = 0.0;
  int target_id = 0;

  if (t_relative > 0.0) {
    // Left lanes: IDs > 0, sorted ascending (e.g. 1, 2, 3...)
    double t_inner = 0.0;
    for (std::uint32_t i = 0; i < lane_cnt_in_sec; ++i) {
      const std::uint32_t lane_idx = first_lane_in_sec + i;
      const int lane_id = lanes_.lane_original_id[lane_idx];
      if (lane_id <= 0) {
        continue;
      }
      const double w = LaneWidth(static_cast<LaneId>(lane_idx), pose.s);
      const double t_outer = t_inner + w;
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
      const std::uint32_t lane_idx = first_lane_in_sec + static_cast<std::uint32_t>(i);
      const int lane_id = lanes_.lane_original_id[lane_idx];
      if (lane_id >= 0) {
        continue;
      }
      const double w = LaneWidth(static_cast<LaneId>(lane_idx), pose.s);
      const double t_outer = t_inner - w;
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

  const double t_lane = t_relative - t_center;
  const double h_offset =
      EvaluateLaneHeightOffset(lanes_, lane_heights_, matched_lane_idx, pose.s, t_lane, target_id, w_target);
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

auto LaneNetwork::LaneToRoad(LanePose pose, QueryContext& /*ctx*/) const noexcept -> RoadPose {
  const auto lane_idx = static_cast<std::uint32_t>(pose.lane);
  if (lane_idx >= lanes_.lane_original_id.size()) {
    return RoadPose{};
  }

  const double s = pose.s;
  const int target_id = lanes_.lane_original_id[lane_idx];
  const RoadId road_id = lanes_.lane_road_id[lane_idx];
  auto road_idx = static_cast<std::uint32_t>(road_id);
  const std::uint32_t sec_idx = lanes_.lane_section_idx[lane_idx];

  // 1. Compute cumulative inner boundary width
  double inner_boundary_t = 0.0;
  const std::uint32_t first_lane_in_sec = lane_sections_.section_first_lane_idx[sec_idx];
  const std::uint32_t lane_cnt_in_sec = lane_sections_.section_lane_count[sec_idx];

  for (std::uint32_t i = 0; i < lane_cnt_in_sec; ++i) {
    const std::uint32_t other_idx = first_lane_in_sec + i;
    const int other_id = lanes_.lane_original_id[other_idx];
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
  const double w_target = LaneWidth(pose.lane, s);

  // 3. Center line t of the lane
  double t_center = 0.0;
  if (target_id > 0) {
    t_center = inner_boundary_t + (0.5 * w_target);
  } else if (target_id < 0) {
    t_center = inner_boundary_t - (0.5 * w_target);
  }

  double road_t = t_center + pose.t;

  const double lane_offset_val = EvaluateRoadLaneOffset(lane_offsets_, road_idx, s);
  road_t += lane_offset_val;

  const double h_offset = EvaluateLaneHeightOffset(lanes_, lane_heights_, lane_idx, s, pose.t, target_id, w_target);
  const double road_h = pose.h + h_offset;

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
  const std::uint32_t w_first = lanes_.lane_first_width_idx[idx];
  const std::uint32_t w_count = lanes_.lane_width_count[idx];
  if (w_count == 0) {
    return 0.0;
  }

  std::uint32_t active_idx = w_first;
  for (std::uint32_t i = 0; i < w_count; ++i) {
    const std::uint32_t cur_idx = w_first + i;
    if (s_coord >= lane_widths_.lane_width_s_start[cur_idx]) {
      active_idx = cur_idx;
    } else {
      break;
    }
  }

  const double ds = s_coord - lane_widths_.lane_width_s_start[active_idx];
  return lane_widths_.lane_width_a[active_idx] + (lane_widths_.lane_width_b[active_idx] * ds) +
         (lane_widths_.lane_width_c[active_idx] * ds * ds) + (lane_widths_.lane_width_d[active_idx] * ds * ds * ds);
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
    const double w = LaneWidth(static_cast<LaneId>(lane_idx), s_coord);
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
    for (std::uint32_t i = 0; i < lo_count; ++i) {
      auto cur_lo = lo_first + i;
      if (s_coord >= lane_offsets_.lane_offset_s_start[cur_lo]) {
        active_lo = cur_lo;
      } else {
        break;
      }
    }
    const double ds_lo = s_coord - lane_offsets_.lane_offset_s_start[active_lo];
    lane_offset_val =
        lane_offsets_.lane_offset_a[active_lo] +
        (ds_lo * (lane_offsets_.lane_offset_b[active_lo] +
                  ds_lo * (lane_offsets_.lane_offset_c[active_lo] + ds_lo * lane_offsets_.lane_offset_d[active_lo])));
  }

  t_left += lane_offset_val;
  t_right += lane_offset_val;
}

auto LaneNetwork::FindLaneId(RoadId road_id, std::uint32_t relative_section_idx, int original_lane_id) const noexcept
    -> std::optional<LaneId> {
  const auto road_idx = static_cast<std::uint32_t>(road_id);
  if (road_idx >= lane_sections_.road_section_first_idx.size()) {
    return std::nullopt;
  }

  const auto road_sec_first = lane_sections_.road_section_first_idx[road_idx];
  const auto road_sec_count = lane_sections_.road_section_count[road_idx];
  if (relative_section_idx >= road_sec_count) {
    return std::nullopt;
  }

  const std::uint32_t abs_sec_idx = road_sec_first + relative_section_idx;
  if (abs_sec_idx >= lane_sections_.section_first_lane_idx.size()) {
    return std::nullopt;
  }

  const std::uint32_t first_lane_idx = lane_sections_.section_first_lane_idx[abs_sec_idx];
  const std::uint32_t lane_count = lane_sections_.section_lane_count[abs_sec_idx];

  for (std::uint32_t i = 0; i < lane_count; ++i) {
    const std::uint32_t lane_idx = first_lane_idx + i;
    if (lane_idx < lanes_.lane_original_id.size()) {
      if (lanes_.lane_original_id[lane_idx] == original_lane_id) {
        return static_cast<LaneId>(lane_idx);
      }
    }
  }

  return std::nullopt;
}

auto LaneNetwork::GetMaxRoadWidth(RoadId road_id, double road_length) const noexcept -> double {
  double max_road_t = 0.0;
  const auto road_idx = static_cast<std::uint32_t>(road_id);
  if (road_idx >= lane_sections_.road_section_first_idx.size()) {
    return 0.0;
  }

  const auto road_sec_first = lane_sections_.road_section_first_idx[road_idx];
  const auto road_sec_count = lane_sections_.road_section_count[road_idx];

  for (std::uint32_t sec_idx = 0; sec_idx < road_sec_count; ++sec_idx) {
    const std::uint32_t cur_sec = road_sec_first + sec_idx;
    const double sec_s_start = lane_sections_.section_s[cur_sec];
    double sec_length = 0.0;
    if (sec_idx + 1 < road_sec_count) {
      sec_length = lane_sections_.section_s[cur_sec + 1] - sec_s_start;
    } else {
      sec_length = road_length - sec_s_start;
    }

    constexpr int sec_samples = 10;
    for (int i = 0; i <= sec_samples; ++i) {
      const double s_local = (static_cast<double>(i) / sec_samples) * sec_length;
      const double s = sec_s_start + s_local;
      double t_left = 0.0;
      double t_right = 0.0;
      GetRoadWidthLimits(road_id, s, t_left, t_right);

      max_road_t = std::max(max_road_t, std::abs(t_left));
      max_road_t = std::max(max_road_t, std::abs(t_right));
    }
  }

  constexpr double road_width_safety_buffer = 0.1;
  return max_road_t + road_width_safety_buffer;
}

}  // namespace strada::cpm
