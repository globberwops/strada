// SPDX-License-Identifier: BSL-1.0

#include "road_projector.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <strada/cpm/elevation_profile.hpp>
#include <strada/cpm/lane_network.hpp>
#include <strada/cpm/reference_line.hpp>

#include "rotation.hpp"

namespace strada::cpm {

RoadProjector::RoadProjector(const ReferenceLine& ref_line, const ElevationProfile& elevation_profile,
                             const LaneNetwork& lane_network) noexcept
    : ref_line_(&ref_line), elevation_profile_(&elevation_profile), lane_network_(&lane_network) {}

auto RoadProjector::Project(RoadId road_id, InertialPose pose, QueryContext& ctx) const noexcept
    -> std::optional<RoadPose> {
  const auto [first_seg, seg_count] = ref_line_->GetRoadSegments(road_id);
  if (seg_count == 0) {
    return std::nullopt;
  }

  auto min_dist_sq = std::numeric_limits<double>::max();
  auto best_s = 0.0;
  auto best_rhdg = 0.0;

  for (auto i = 0U; i < seg_count; ++i) {
    const auto seg_idx = first_seg + i;
    const auto road_s = ref_line_->Project(seg_idx, pose.x, pose.y);
    const auto pt = ref_line_->Evaluate(seg_idx, road_s);

    const auto dx = pose.x - pt.x;
    const auto dy = pose.y - pt.y;
    const auto dist_sq = (dx * dx) + (dy * dy);
    if (dist_sq < min_dist_sq) {
      min_dist_sq = dist_sq;
      best_s = road_s;
      best_rhdg = pt.heading;
    }
  }

  // Evaluate base vertical profile at best_s (t=0)
  const auto vertical_base = elevation_profile_->Evaluate(road_id, best_s, 0.0);

  const auto best_seg_idx = ref_line_->FindSegmentIndex(road_id, best_s, ctx);
  const auto pt = ref_line_->Evaluate(best_seg_idx, best_s);

  const auto dx = pose.x - pt.x;
  const auto dy = pose.y - pt.y;
  const auto dz = pose.z - vertical_base.elevation;

  // Base roll calculation
  const auto r_road_base = Rotation::FromEuler(best_rhdg, vertical_base.pitch, vertical_base.natural_roll);
  const auto road_t_base = r_road_base.InverseTransform(dx, dy, dz)[1];

  // Shape evaluation and roll correction
  const auto shape_grad = elevation_profile_->EvaluateShapeTGradient(road_id, best_s, road_t_base);
  const auto roll_total = vertical_base.natural_roll + std::atan(shape_grad);

  const auto r_road = Rotation::FromEuler(best_rhdg, vertical_base.pitch, roll_total);
  const auto road_t = r_road.InverseTransform(dx, dy, dz)[1];

  auto t_left = 0.0;
  auto t_right = 0.0;
  lane_network_->GetRoadWidthLimits(road_id, best_s, t_left, t_right);

  const auto ds_longitudinal = std::sqrt(std::max(0.0, min_dist_sq - (road_t * road_t)));
  if (ds_longitudinal > kSnappingTolerance) {
    return std::nullopt;
  }

  if (road_t >= t_right - kSnappingTolerance && road_t <= t_left + kSnappingTolerance) {
    const auto h_surf = lane_network_->EvaluateCrossSectionSurfaceOffset(road_id, best_s, road_t);
    const auto h_shape = elevation_profile_->EvaluateShapeHeight(road_id, best_s, road_t);

    const auto local_h = r_road.InverseTransform(dx, dy, dz)[2];
    const auto h_total = local_h - h_surf - h_shape;

    const auto r_inertial = Rotation::FromEuler(pose.heading, pose.pitch, pose.roll);
    const auto r_offset = r_road.Inverse().Compose(r_inertial);
    const auto offset_angles = r_offset.ToEuler();

    RoadPose road_pose{.s = best_s,
                       .t = road_t,
                       .h = h_total,
                       .heading = offset_angles.heading,
                       .pitch = offset_angles.pitch,
                       .roll = offset_angles.roll,
                       .road = road_id};
    return road_pose;
  }
  return std::nullopt;
}

}  // namespace strada::cpm
