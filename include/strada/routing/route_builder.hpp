// This file declares the RouteBuilder utility class.
// It manages the incremental planning and state of a Route.

#pragma once

#include <optional>
#include <strada/routing/graph.hpp>
#include <string>
#include <string_view>
#include <vector>

namespace strada::routing {

/// Manages the state, waypoint history, and incremental calculation of a Route.
///
/// RouteBuilder acts as a stateful coordinator for path planning. It tracks
/// the sequence of selected waypoints, plans paths between adjacent pairs,
/// caches successfully calculated sub-routes to optimize history navigation,
/// and maintains pathfinding error status.
class RouteBuilder {
 public:
  /// Constructs a RouteBuilder associated with the given Graph.
  ///
  /// \param graph The Graph topology to query for path planning.
  explicit RouteBuilder(const Graph& graph);
  ~RouteBuilder() = default;

  // Disable copying.
  RouteBuilder(const RouteBuilder&) = delete;
  RouteBuilder& operator=(const RouteBuilder&) = delete;

  // Enable moving.
  RouteBuilder(RouteBuilder&&) noexcept = default;
  RouteBuilder& operator=(RouteBuilder&&) noexcept = default;

  /// Appends a new waypoint to the sequence and plans the path from the last waypoint.
  ///
  /// \param road_id The ID of the road to add as a waypoint.
  /// \return True if the segment was successfully planned or if it is the first waypoint,
  ///         false if pathfinding failed.
  auto AppendWaypoint(std::string_view road_id) -> bool;

  /// Removes the last added waypoint and restores the previous route state in O(1).
  void Undo();

  /// Clears all waypoints, active route segments, and errors.
  void Clear();

  /// Returns the current list of waypoint road IDs.
  ///
  /// \return A reference to the sequence of waypoints.
  auto Waypoints() const -> const std::vector<std::string>&;

  /// Returns the active computed Route.
  ///
  /// \return The active Route, or std::nullopt if the route is uncompleted or failed.
  auto ActiveRoute() const -> const std::optional<Route>&;

  /// Returns the details of the last pathfinding error.
  ///
  /// \return The error string, or empty if there is no active error.
  auto RouteError() const -> std::string_view;

 private:
  const Graph& graph_;
  std::vector<std::string> waypoint_road_ids_;
  std::vector<Route> sub_routes_;
  std::optional<Route> active_route_;
  std::string route_error_;
  std::vector<std::optional<Route>> route_history_;
  std::vector<std::string> error_history_;
};

}  // namespace strada::routing
