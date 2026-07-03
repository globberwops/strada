// SPDX-License-Identifier: BSL-1.0

#include <algorithm>
#include <cmath>
#include <limits>
#include <span>
#include <strada/cpm/compiled_physics_model.hpp>
#include <strada/cpm/geometry_math.hpp>
#include <vector>

#include "rotation.hpp"

namespace strada::cpm {

namespace {

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

}  // namespace

auto CompiledPhysicsModel::Build(const ast::AbstractSyntaxTree& map) -> CompiledPhysicsModel {
  CompiledPhysicsModel model;
  model.ref_line_ = ReferenceLine::Build(map);
  model.elevation_profile_ = ElevationProfile::Build(map);
  model.lane_network_ = LaneNetwork::Build(map);

  for (const auto& road : map.roads) {
    model.road_string_ids_.push_back(road.id);
    model.road_lengths_.push_back(road.length);
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

  // 3. Evaluate vertical profile (elevation, pitch, roll, shape height)
  auto vertical = elevation_profile_.Evaluate(pose.road, pose.s, pose.t);

  // 4. Cross section surface height offset
  double h_surf = lane_network_.EvaluateCrossSectionSurfaceOffset(pose.road, pose.s, pose.t);

  // 5. Position composition
  auto r_road = Rotation::FromEuler(pt.heading, vertical.pitch, vertical.roll_total);

  double local_t = pose.t;
  double local_h = pose.h + h_surf + vertical.shape_height;

  auto offset = r_road.Transform(0.0, local_t, local_h);

  InertialPose inertial_pose;
  inertial_pose.x = pt.x + offset[0];
  inertial_pose.y = pt.y + offset[1];
  inertial_pose.z = vertical.elevation + offset[2];

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
      double road_s = ref_line_.Project(seg_idx, pose.x, pose.y);
      auto pt = ref_line_.Evaluate(seg_idx, road_s);

      double dx = pose.x - pt.x;
      double dy = pose.y - pt.y;
      double dist_sq = (dx * dx) + (dy * dy);
      if (dist_sq < min_dist_sq) {
        min_dist_sq = dist_sq;
        best_s = road_s;
        best_t = (-dx * std::sin(pt.heading)) + (dy * std::cos(pt.heading));
        best_rhdg = pt.heading;
      }
    }

    // Evaluate base vertical profile at best_s (t=0)
    auto vertical_base = elevation_profile_.Evaluate(static_cast<RoadId>(road_idx), best_s, 0.0);

    uint32_t best_seg_idx = ref_line_.FindSegmentIndex(static_cast<RoadId>(road_idx), best_s, ctx);
    auto pt = ref_line_.Evaluate(best_seg_idx, best_s);

    double dx = pose.x - pt.x;
    double dy = pose.y - pt.y;
    double dz = pose.z - vertical_base.elevation;

    // Base roll calculation
    auto r_road_base = Rotation::FromEuler(best_rhdg, vertical_base.pitch, vertical_base.natural_roll);
    double road_t_base = r_road_base.InverseTransform(dx, dy, dz)[1];

    // Shape evaluation and roll correction
    double shape_grad = elevation_profile_.EvaluateShapeTGradient(static_cast<RoadId>(road_idx), best_s, road_t_base);
    double roll_total = vertical_base.natural_roll + std::atan(shape_grad);

    auto r_road = Rotation::FromEuler(best_rhdg, vertical_base.pitch, roll_total);
    double road_t = r_road.InverseTransform(dx, dy, dz)[1];

    double t_left = 0.0;
    double t_right = 0.0;
    lane_network_.GetRoadWidthLimits(static_cast<RoadId>(road_idx), best_s, t_left, t_right);
    constexpr double kSnappingTolerance = 5.0;

    double ds_longitudinal =
        std::sqrt(std::max(0.0, min_dist_sq - (road_t * road_t)));
    if (ds_longitudinal > kSnappingTolerance) {
      return std::nullopt;
    }

    if (road_t >= t_right - kSnappingTolerance && road_t <= t_left + kSnappingTolerance) {
      RoadPose road_pose;
      road_pose.road = static_cast<RoadId>(road_idx);
      road_pose.s = best_s;
      road_pose.t = road_t;

      double h_surf = lane_network_.EvaluateCrossSectionSurfaceOffset(static_cast<RoadId>(road_idx), best_s, road_t);
      double h_shape = elevation_profile_.EvaluateShapeHeight(static_cast<RoadId>(road_idx), best_s, road_t);

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
  return lane_network_.RoadToLane(pose, ctx);
}

auto CompiledPhysicsModel::LaneToRoad(LanePose pose, QueryContext& ctx) const noexcept -> RoadPose {
  return lane_network_.LaneToRoad(pose, ctx);
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

auto CompiledPhysicsModel::LaneCount() const noexcept -> std::size_t { return lane_network_.LaneCount(); }

auto CompiledPhysicsModel::LaneRoad(LaneId lane_id) const noexcept -> RoadId { return lane_network_.LaneRoad(lane_id); }

auto CompiledPhysicsModel::OriginalLaneId(LaneId lane_id) const noexcept -> int {
  return lane_network_.OriginalLaneId(lane_id);
}

auto CompiledPhysicsModel::LaneWidth(LaneId lane_id, double s_coord) const noexcept -> double {
  return lane_network_.LaneWidth(lane_id, s_coord);
}

}  // namespace strada::cpm
