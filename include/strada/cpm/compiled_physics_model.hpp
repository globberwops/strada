// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <cstddef>
#include <optional>
#include <strada/ast/abstract_syntax_tree.hpp>
#include <strada/cpm/bounding_volume_hierarchy.hpp>
#include <strada/cpm/coordinate.hpp>
#include <strada/cpm/elevation_profile.hpp>
#include <strada/cpm/ids.hpp>
#include <strada/cpm/lane_network.hpp>
#include <strada/cpm/query_context.hpp>
#include <strada/cpm/reference_line.hpp>
#include <string>
#include <vector>

namespace strada::cpm {

/// Optimised, flat representation of the road network physics model.
///
/// This class holds compiled representation of plan-view geometries, lane sections,
/// lane profiles, and lateral cross-sections. It is designed to expose hot-path
/// coordinate conversions and queries in a performant, thread-safe manner.
class CompiledPhysicsModel {
 public:
  /// Default-constructs a CompiledPhysicsModel.
  CompiledPhysicsModel() = default;

  /// Destructs a CompiledPhysicsModel.
  ~CompiledPhysicsModel() = default;

  // Move-constructible only, per ADR 0004
  CompiledPhysicsModel(const CompiledPhysicsModel&) = delete;
  auto operator=(const CompiledPhysicsModel&) -> CompiledPhysicsModel& = delete;
  CompiledPhysicsModel(CompiledPhysicsModel&&) noexcept = default;
  auto operator=(CompiledPhysicsModel&&) noexcept -> CompiledPhysicsModel& = default;

  /// Compiles the abstract syntax tree (AST) into a highly optimised CompiledPhysicsModel.
  ///
  /// \param map The parsed AST of the road network map.
  /// \return The compiled physics model ready for query execution.
  static auto Build(const ast::AbstractSyntaxTree& map) -> CompiledPhysicsModel;

  /// Converts a pose from road-local coordinates (s, t, h) to global inertial coordinates (x, y, z).
  ///
  /// \param pose The source pose in road-local coordinates.
  /// \param ctx The query context used to exploit temporal coherence.
  /// \return The resulting global inertial pose.
  auto RoadToInertial(RoadPose pose, QueryContext& ctx) const noexcept -> InertialPose;

  /// Converts a pose from lane-local coordinates (s, t, h) to global inertial coordinates (x, y, z).
  ///
  /// \param pose The source pose in lane-local coordinates.
  /// \param ctx The query context used to exploit temporal coherence.
  /// \return The resulting global inertial pose.
  auto LaneToInertial(LanePose pose, QueryContext& ctx) const noexcept -> InertialPose;

  /// Converts a pose from global inertial coordinates (x, y, z) to road-local coordinates (s, t, h).
  ///
  /// \param pose The source pose in global inertial coordinates.
  /// \param ctx The query context used to exploit temporal coherence.
  /// \return The matched road pose, or std::nullopt if the point is outside the spatial index.
  auto InertialToRoad(InertialPose pose, QueryContext& ctx) const noexcept -> std::optional<RoadPose>;

  /// Converts a pose from global inertial coordinates (x, y, z) to lane-local coordinates (s, t, h).
  ///
  /// \param pose The source pose in global inertial coordinates.
  /// \param ctx The query context used to exploit temporal coherence.
  /// \return The matched lane pose, or std::nullopt if the point is outside the spatial index.
  auto InertialToLane(InertialPose pose, QueryContext& ctx) const noexcept -> std::optional<LanePose>;

  /// Translates a road-local pose to the corresponding lane-local pose.
  ///
  /// \param pose The source pose in road-local coordinates.
  /// \param ctx The query context used to exploit temporal coherence.
  /// \return The matched lane pose, or std::nullopt if the point does not lie in any lane.
  auto RoadToLane(RoadPose pose, QueryContext& ctx) const noexcept -> std::optional<LanePose>;

  /// Translates a lane-local pose to the corresponding road-local pose.
  ///
  /// \param pose The source pose in lane-local coordinates.
  /// \param ctx The query context used to exploit temporal coherence.
  /// \return The corresponding road-local pose.
  auto LaneToRoad(LanePose pose, QueryContext& ctx) const noexcept -> RoadPose;

  /// Returns the total number of roads compile-mapped in the network.
  ///
  /// \return The number of roads.
  [[nodiscard]] auto RoadCount() const noexcept -> std::size_t;

  /// Resolves the integer RoadId corresponding to the parser's road string ID.
  ///
  /// \param original_id The road string ID parsed from the XODR source.
  /// \return The unique RoadId if found, or std::nullopt if not mapped.
  [[nodiscard]] auto RoadIdFromString(std::string_view original_id) const noexcept -> std::optional<RoadId>;

  /// Retrieves the original road string ID corresponding to the numeric RoadId.
  ///
  /// \param road_id The compiled RoadId.
  /// \return The XODR road string ID.
  [[nodiscard]] auto OriginalRoadId(RoadId road_id) const noexcept -> std::string_view;

  /// Returns the reference line length of the specified road.
  ///
  /// \param road_id The compiled RoadId.
  /// \return The road's length.
  [[nodiscard]] auto RoadLength(RoadId road_id) const noexcept -> double;

  /// Returns the total number of lanes compile-mapped in the network.
  ///
  /// \return The number of lanes.
  [[nodiscard]] auto LaneCount() const noexcept -> std::size_t;

  /// Retrieves the parent RoadId of the specified compiled LaneId.
  ///
  /// \param lane_id The compiled LaneId.
  /// \return The parent road's RoadId.
  [[nodiscard]] auto LaneRoad(LaneId lane_id) const noexcept -> RoadId;

  /// Retrieves the original XODR lane integer ID corresponding to the numeric LaneId.
  ///
  /// \param lane_id The compiled LaneId.
  /// \return The original lane ID from the XODR file.
  [[nodiscard]] auto OriginalLaneId(LaneId lane_id) const noexcept -> int;

  /// Computes the width of the specified lane at a given s-coordinate along its parent road.
  ///
  /// \param lane_id The compiled LaneId.
  /// \param s_coord The s-coordinate along the reference line.
  /// \return The computed lane width.
  [[nodiscard]] auto LaneWidth(LaneId lane_id, double s_coord) const noexcept -> double;

  /// Returns a reference to the flat vector of nodes in the spatial bounding volume hierarchy.
  ///
  /// \return The contiguous array of hierarchy nodes.
  [[nodiscard]] auto GetBoundingVolumeHierarchyNodes() const noexcept
      -> const std::vector<BoundingVolumeHierarchy::Node>& {
    return bounding_volume_hierarchy_.Nodes();
  }

  /// Returns a reference to the flat vector of leaf primitives in the spatial bounding volume hierarchy.
  ///
  /// \return The contiguous array of leaf primitives.
  [[nodiscard]] auto GetBoundingVolumeHierarchyPrimitives() const noexcept
      -> const std::vector<BoundingVolumeHierarchy::PrimitiveInfo>& {
    return bounding_volume_hierarchy_.Primitives();
  }

  /// Clears the bounding volume hierarchy node and primitive tables.
  void ClearBoundingVolumeHierarchyNodes() noexcept { bounding_volume_hierarchy_.Clear(); }

 private:
  std::vector<std::string> road_string_ids_;
  std::vector<double> road_lengths_;

  ReferenceLine ref_line_;
  ElevationProfile elevation_profile_;
  LaneNetwork lane_network_;
  BoundingVolumeHierarchy bounding_volume_hierarchy_;
};
}  // namespace strada::cpm
