#pragma once

#include <optional>
#include <strada/cpm/coordinate.hpp>
#include <strada/cpm/ids.hpp>
#include <strada/cpm/query_context.hpp>

namespace strada::cpm {

class ReferenceLine;
class ElevationProfile;
class LaneNetwork;

/// Provides coordinate projection and road snapping calculations.
class RoadProjector {
 public:
  /// Distance threshold for snapping a coordinate onto a road.
  static constexpr auto kSnappingTolerance = 5.0;

  /// Constructs a RoadProjector with references to required components.
  RoadProjector(const ReferenceLine& ref_line, const ElevationProfile& elevation_profile,
                const LaneNetwork& lane_network) noexcept;

  /// Projects a global inertial pose onto a specific road.
  ///
  /// \param road_id The ID of the road to project onto.
  /// \param pose The global inertial pose.
  /// \param ctx The query context for temporal coherence.
  /// \return The resulting road-local pose, or std::nullopt if the pose does not snap to the road.
  auto Project(RoadId road_id, InertialPose pose, QueryContext& ctx) const noexcept -> std::optional<RoadPose>;

 private:
  const ReferenceLine* ref_line_{};
  const ElevationProfile* elevation_profile_{};
  const LaneNetwork* lane_network_{};
};

}  // namespace strada::cpm
