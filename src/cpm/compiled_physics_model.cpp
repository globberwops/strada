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
  const double kDs = s_local_to_section - active->s_offset;
  return active->a + (active->b * kDs) + (active->c * kDs * kDs) + (active->d * kDs * kDs * kDs);
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
    for (std::size_t sec_idx = 0; sec_idx < road.lanes.sections.size(); ++sec_idx) {
      const auto& section = road.lanes.sections[sec_idx];
      double sec_length = 0.0;
      if (sec_idx + 1 < road.lanes.sections.size()) {
        sec_length = road.lanes.sections[sec_idx + 1].s - section.s;
      } else {
        sec_length = road.length - section.s;
      }

      constexpr int kSecSamples = 10;
      for (int i = 0; i <= kSecSamples; ++i) {
        const double kSLocal = (static_cast<double>(i) / kSecSamples) * sec_length;
        double left_width = 0.0;
        double right_width = 0.0;

        for (const auto& lane : section.left) {
          left_width += EvaluateAstLaneWidth(lane, kSLocal);
        }
        for (const auto& lane : section.right) {
          right_width += EvaluateAstLaneWidth(lane, kSLocal);
        }

        // Evaluate road laneOffset
        double lane_offset_val = 0.0;
        if (!road.lanes.offsets.empty()) {
          const ast::LaneOffset* active = road.lanes.offsets.data();
          const double kSRoad = section.s + kSLocal;
          for (const auto& offset : road.lanes.offsets) {
            if (kSRoad >= offset.s) {
              active = &offset;
            } else {
              break;
            }
          }
          const double kDsOffset = kSRoad - active->s;
          lane_offset_val = active->a + (kDsOffset * (active->b + (kDsOffset * (active->c + (kDsOffset * active->d)))));
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

  const auto kNumRoads = static_cast<std::uint32_t>(model.road_lengths_.size());
  for (std::uint32_t road_idx = 0; road_idx < kNumRoads; ++road_idx) {
    const auto [first_seg, seg_count] = model.ref_line_.GetRoadSegments(static_cast<RoadId>(road_idx));
    const auto kInflation = road_max_t[road_idx];
    for (std::uint32_t i = 0; i < seg_count; ++i) {
      const std::uint32_t kSegIdx = first_seg + i;
      temp_primitives.push_back(BoundingVolumeHierarchy::PrimitiveInfo{.road_idx = road_idx, .segment_idx = kSegIdx});
      const auto kAabb = model.ref_line_.ComputeSegmentAabb(kSegIdx, kInflation);
      temp_aabbs.push_back(kAabb);
    }
  }

  if (!temp_primitives.empty()) {
    std::vector<std::uint32_t> prim_indices(temp_primitives.size());
    for (std::uint32_t i = 0; i < prim_indices.size(); ++i) {
      prim_indices[i] = i;
    }
    model.bounding_volume_hierarchy_ = BoundingVolumeHierarchy::Build(prim_indices, temp_primitives, temp_aabbs);
  }

  return model;
}

[[gnu::hot]] auto CompiledPhysicsModel::RoadToInertial(RoadPose pose, QueryContext& ctx) const noexcept
    -> InertialPose {
  const auto kRoadIdx = static_cast<std::uint32_t>(pose.road);
  const auto [first_seg, seg_count] = ref_line_.GetRoadSegments(pose.road);
  if (seg_count == 0) {
    return InertialPose{};
  }

  // 1. Find segment index
  const std::uint32_t kSegIdx = ref_line_.FindSegmentIndex(pose.road, pose.s, ctx);

  // 2. Evaluate reference line
  const auto kPt = ref_line_.Evaluate(kSegIdx, pose.s);

  // 3. Evaluate vertical profile (elevation, pitch, roll, shape height)
  const auto kVertical = elevation_profile_.Evaluate(pose.road, pose.s, pose.t);

  // 4. Cross section surface height offset
  const double kHSurf = lane_network_.EvaluateCrossSectionSurfaceOffset(pose.road, pose.s, pose.t);

  // 5. Position composition
  const auto kRRoad = Rotation::FromEuler(kPt.heading, kVertical.pitch, kVertical.roll_total);

  const double kLocalT = pose.t;
  const double kLocalH = pose.h + kHSurf + kVertical.shape_height;

  const auto kOffset = kRRoad.Transform(0.0, kLocalT, kLocalH);

  InertialPose inertial_pose;
  inertial_pose.x = kPt.x + kOffset[0];
  inertial_pose.y = kPt.y + kOffset[1];
  inertial_pose.z = kVertical.elevation + kOffset[2];

  // Composed orientation composition
  const auto kROffset = Rotation::FromEuler(pose.heading, pose.pitch, pose.roll);
  const auto kRInertial = kRRoad.Compose(kROffset);

  const auto kEulerAngles = kRInertial.ToEuler();
  inertial_pose.heading = kEulerAngles.heading;
  inertial_pose.pitch = kEulerAngles.pitch;
  inertial_pose.roll = kEulerAngles.roll;

  return inertial_pose;
}

auto CompiledPhysicsModel::LaneToInertial(LanePose pose, QueryContext& ctx) const noexcept -> InertialPose {
  const RoadPose kRoadPose = LaneToRoad(pose, ctx);
  return RoadToInertial(kRoadPose, ctx);
}

auto CompiledPhysicsModel::InertialToRoad(InertialPose pose, QueryContext& ctx) const noexcept
    -> std::optional<RoadPose> {
  auto snap_to_road = [&](std::uint32_t road_idx) noexcept -> std::optional<RoadPose> {
    auto [first_seg, seg_count] = ref_line_.GetRoadSegments(static_cast<RoadId>(road_idx));
    if (seg_count == 0) {
      return std::nullopt;
    }

    double min_dist_sq = std::numeric_limits<double>::max();
    double best_s = 0.0;
    double best_t = 0.0;
    double best_rhdg = 0.0;

    for (std::uint32_t i = 0; i < seg_count; ++i) {
      const std::uint32_t kSegIdx = first_seg + i;
      const double kRoadS = ref_line_.Project(kSegIdx, pose.x, pose.y);
      const auto kPt = ref_line_.Evaluate(kSegIdx, kRoadS);

      const double kDx = pose.x - kPt.x;
      const double kDy = pose.y - kPt.y;
      const double kDistSq = (kDx * kDx) + (kDy * kDy);
      if (kDistSq < min_dist_sq) {
        min_dist_sq = kDistSq;
        best_s = kRoadS;
        best_t = (-kDx * std::sin(kPt.heading)) + (kDy * std::cos(kPt.heading));
        best_rhdg = kPt.heading;
      }
    }

    // Evaluate base vertical profile at best_s (t=0)
    const auto kVerticalBase = elevation_profile_.Evaluate(static_cast<RoadId>(road_idx), best_s, 0.0);

    const std::uint32_t kBestSegIdx = ref_line_.FindSegmentIndex(static_cast<RoadId>(road_idx), best_s, ctx);
    const auto kPt = ref_line_.Evaluate(kBestSegIdx, best_s);

    const double kDx = pose.x - kPt.x;
    const double kDy = pose.y - kPt.y;
    const double kDz = pose.z - kVerticalBase.elevation;

    // Base roll calculation
    const auto kRRoadBase = Rotation::FromEuler(best_rhdg, kVerticalBase.pitch, kVerticalBase.natural_roll);
    const double kRoadTBase = kRRoadBase.InverseTransform(kDx, kDy, kDz)[1];

    // Shape evaluation and roll correction
    const double kShapeGrad =
        elevation_profile_.EvaluateShapeTGradient(static_cast<RoadId>(road_idx), best_s, kRoadTBase);
    const double kRollTotal = kVerticalBase.natural_roll + std::atan(kShapeGrad);

    const auto kRRoad = Rotation::FromEuler(best_rhdg, kVerticalBase.pitch, kRollTotal);
    const double kRoadT = kRRoad.InverseTransform(kDx, kDy, kDz)[1];

    double t_left = 0.0;
    double t_right = 0.0;
    lane_network_.GetRoadWidthLimits(static_cast<RoadId>(road_idx), best_s, t_left, t_right);
    constexpr double kSnappingTolerance = 5.0;

    const double kDsLongitudinal = std::sqrt(std::max(0.0, min_dist_sq - (kRoadT * kRoadT)));
    if (kDsLongitudinal > kSnappingTolerance) {
      return std::nullopt;
    }

    if (kRoadT >= t_right - kSnappingTolerance && kRoadT <= t_left + kSnappingTolerance) {
      const double kHSurf =
          lane_network_.EvaluateCrossSectionSurfaceOffset(static_cast<RoadId>(road_idx), best_s, kRoadT);
      const double kHShape = elevation_profile_.EvaluateShapeHeight(static_cast<RoadId>(road_idx), best_s, kRoadT);

      const double kLocalH = kRRoad.InverseTransform(kDx, kDy, kDz)[2];
      const double kHTotal = kLocalH - kHSurf - kHShape;

      const auto kRInertial = Rotation::FromEuler(pose.heading, pose.pitch, pose.roll);
      const auto kROffset = kRRoad.Inverse().Compose(kRInertial);
      const auto kOffsetAngles = kROffset.ToEuler();

      RoadPose road_pose{.s = best_s,
                         .t = kRoadT,
                         .h = kHTotal,
                         .heading = kOffsetAngles.heading,
                         .pitch = kOffsetAngles.pitch,
                         .roll = kOffsetAngles.roll,
                         .road = static_cast<RoadId>(road_idx)};
      return road_pose;
    }
    return std::nullopt;
  };

  // 1. Check temporal coherence fast path
  if (ctx.last_road.has_value()) {
    const auto kRoadIdx = static_cast<std::uint32_t>(*ctx.last_road);
    const auto kFastPose = snap_to_road(kRoadIdx);
    if (kFastPose.has_value()) {
      ref_line_.FindSegmentIndex(*ctx.last_road, kFastPose->s, ctx);
      return kFastPose;
    }
  }

  // 2. Traversal stack-based bounding volume hierarchy search
  std::optional<RoadPose> best_overall_pose;

  bounding_volume_hierarchy_.Query(
      pose.x, pose.y,
      [&](const BoundingVolumeHierarchy::PrimitiveInfo& prim, double current_min_dist) -> std::optional<double> {
        const auto kCandidate = snap_to_road(prim.road_idx);
        if (kCandidate.has_value()) {
          double abs_t = std::abs(kCandidate->t);
          if (abs_t < current_min_dist) {
            best_overall_pose = kCandidate;
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
  const auto kRoadPoseOpt = InertialToRoad(pose, ctx);
  if (!kRoadPoseOpt.has_value()) {
    return std::nullopt;
  }
  return RoadToLane(*kRoadPoseOpt, ctx);
}

auto CompiledPhysicsModel::RoadToLane(RoadPose pose, QueryContext& ctx) const noexcept -> std::optional<LanePose> {
  return lane_network_.RoadToLane(pose, ctx);
}

auto CompiledPhysicsModel::LaneToRoad(LanePose pose, QueryContext& ctx) const noexcept -> RoadPose {
  return lane_network_.LaneToRoad(pose, ctx);
}

auto CompiledPhysicsModel::RoadCount() const noexcept -> std::size_t { return road_string_ids_.size(); }

auto CompiledPhysicsModel::RoadIdFromString(std::string_view original_id) const noexcept -> std::optional<RoadId> {
  const auto kFindIt = std::ranges::find(road_string_ids_, original_id);
  if (kFindIt != road_string_ids_.end()) {
    return static_cast<RoadId>(std::distance(road_string_ids_.begin(), kFindIt));
  }
  return std::nullopt;
}

auto CompiledPhysicsModel::OriginalRoadId(RoadId road_id) const noexcept -> std::string_view {
  const auto kIdx = static_cast<std::uint32_t>(road_id);
  if (kIdx < road_string_ids_.size()) {
    return road_string_ids_[kIdx];
  }
  return "";
}

auto CompiledPhysicsModel::RoadLength(RoadId road_id) const noexcept -> double {
  const auto kIdx = static_cast<std::uint32_t>(road_id);
  if (kIdx < road_lengths_.size()) {
    return road_lengths_[kIdx];
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
