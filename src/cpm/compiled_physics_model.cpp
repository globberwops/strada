#include <algorithm>
#include <cmath>
#include <limits>
#include <span>
#include <strada/cpm/compiled_physics_model.hpp>
#include <strada/cpm/geometry_math.hpp>
#include <vector>

#include "road_projector.hpp"
#include "rotation.hpp"

namespace strada::cpm {

namespace {}  // namespace

CompiledPhysicsModel::CompiledPhysicsModel(const ast::AbstractSyntaxTree& map)
    : ref_line_(map), elevation_profile_(map), lane_network_(map) {
  for (const auto& road : map.roads) {
    road_string_ids_.push_back(road.id);
    road_lengths_.push_back(road.length);
  }

  // Global bounding volume hierarchy construction
  std::vector<double> road_max_t;
  road_max_t.reserve(map.roads.size());

  for (std::size_t road_idx = 0; road_idx < map.roads.size(); ++road_idx) {
    const double max_road_t = lane_network_.GetMaxRoadWidth(static_cast<RoadId>(road_idx), road_lengths_[road_idx]);
    road_max_t.push_back(max_road_t);
  }

  std::vector<BoundingVolumeHierarchy::PrimitiveInfo> temp_primitives;
  std::vector<Aabb> temp_aabbs;

  const auto num_roads = static_cast<std::uint32_t>(road_lengths_.size());
  for (std::uint32_t road_idx = 0; road_idx < num_roads; ++road_idx) {
    const auto [first_seg, seg_count] = ref_line_.GetRoadSegments(static_cast<RoadId>(road_idx));
    const auto inflation = road_max_t[road_idx];
    for (std::uint32_t i = 0; i < seg_count; ++i) {
      const std::uint32_t seg_idx = first_seg + i;
      temp_primitives.push_back(BoundingVolumeHierarchy::PrimitiveInfo{.road_idx = road_idx, .segment_idx = seg_idx});
      const auto aabb = ref_line_.ComputeSegmentAabb(seg_idx, inflation);
      temp_aabbs.push_back(aabb);
    }
  }

  if (!temp_primitives.empty()) {
    std::vector<std::uint32_t> prim_indices(temp_primitives.size());
    for (std::uint32_t i = 0; i < prim_indices.size(); ++i) {
      prim_indices[i] = i;
    }
    bounding_volume_hierarchy_ = BoundingVolumeHierarchy(prim_indices, temp_primitives, temp_aabbs);
  }
}

[[gnu::hot]] auto CompiledPhysicsModel::RoadToInertial(RoadPose pose, QueryContext& ctx) const noexcept
    -> InertialPose {
  const auto [first_seg, seg_count] = ref_line_.GetRoadSegments(pose.road);
  if (seg_count == 0) {
    return InertialPose{};
  }

  // 1. Find segment index
  const std::uint32_t seg_idx = ref_line_.FindSegmentIndex(pose.road, pose.s, ctx);

  // 2. Evaluate reference line
  const auto pt = ref_line_.Evaluate(seg_idx, pose.s);

  // 3. Evaluate vertical profile (elevation, pitch, roll, shape height)
  const auto vertical = elevation_profile_.Evaluate(pose.road, pose.s, pose.t);

  // 4. Cross section surface height offset
  const double h_surf = lane_network_.EvaluateCrossSectionSurfaceOffset(pose.road, pose.s, pose.t);

  // 5. Position composition
  const auto r_road = Rotation::FromEuler(pt.heading, vertical.pitch, vertical.roll_total);

  const double local_t = pose.t;
  const double local_h = pose.h + h_surf + vertical.shape_height;

  const auto offset = r_road.Transform(0.0, local_t, local_h);

  InertialPose inertial_pose;
  inertial_pose.x = pt.x + offset[0];
  inertial_pose.y = pt.y + offset[1];
  inertial_pose.z = vertical.elevation + offset[2];

  // Composed orientation composition
  const auto r_offset = Rotation::FromEuler(pose.heading, pose.pitch, pose.roll);
  const auto r_inertial = r_road.Compose(r_offset);

  const auto euler_angles = r_inertial.ToEuler();
  inertial_pose.heading = euler_angles.heading;
  inertial_pose.pitch = euler_angles.pitch;
  inertial_pose.roll = euler_angles.roll;

  return inertial_pose;
}

auto CompiledPhysicsModel::LaneToInertial(LanePose pose, QueryContext& ctx) const noexcept -> InertialPose {
  const RoadPose road_pose = LaneToRoad(pose, ctx);
  return RoadToInertial(road_pose, ctx);
}

auto CompiledPhysicsModel::InertialToRoad(InertialPose pose, QueryContext& ctx) const noexcept
    -> std::optional<RoadPose> {
  const RoadProjector projector(ref_line_, elevation_profile_, lane_network_);

  // 1. Check temporal coherence fast path
  if (ctx.last_road.has_value()) {
    const auto fast_pose = projector.Project(*ctx.last_road, pose, ctx);
    if (fast_pose.has_value()) {
      return fast_pose;
    }
  }

  // 2. Traversal stack-based bounding volume hierarchy search
  std::optional<RoadPose> best_overall_pose;

  bounding_volume_hierarchy_.Query(
      pose.x, pose.y,
      [&](const BoundingVolumeHierarchy::PrimitiveInfo& prim, double current_min_dist) -> std::optional<double> {
        const auto candidate = projector.Project(static_cast<RoadId>(prim.road_idx), pose, ctx);
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
  const auto road_pose_opt = InertialToRoad(pose, ctx);
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
  const auto find_it = std::ranges::find(road_string_ids_, original_id);
  if (find_it != road_string_ids_.end()) {
    return static_cast<RoadId>(std::distance(road_string_ids_.begin(), find_it));
  }
  return std::nullopt;
}

auto CompiledPhysicsModel::OriginalRoadId(RoadId road_id) const noexcept -> std::string_view {
  const auto idx = static_cast<std::uint32_t>(road_id);
  if (idx < road_string_ids_.size()) {
    return road_string_ids_[idx];
  }
  return "";
}

auto CompiledPhysicsModel::RoadLength(RoadId road_id) const noexcept -> double {
  const auto idx = static_cast<std::uint32_t>(road_id);
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

auto CompiledPhysicsModel::FindLaneId(RoadId road_id, std::uint32_t relative_section_idx,
                                      int original_lane_id) const noexcept -> std::optional<LaneId> {
  return lane_network_.FindLaneId(road_id, relative_section_idx, original_lane_id);
}

}  // namespace strada::cpm
