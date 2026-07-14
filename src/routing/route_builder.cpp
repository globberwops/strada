// This file implements the RouteBuilder utility class.

#include "strada/routing/route_builder.hpp"

#include <optional>
#include <strada/routing/graph.hpp>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace strada::routing {

RouteBuilder::RouteBuilder(const Graph& graph) : graph_(graph) {}

auto RouteBuilder::AppendWaypoint(std::string_view road_id) -> bool {
  waypoint_road_ids_.push_back(std::string(road_id));

  if (waypoint_road_ids_.size() < 2) {
    active_route_ = std::nullopt;
    route_error_.clear();
    return true;
  }

  const auto& from = waypoint_road_ids_[waypoint_road_ids_.size() - 2];
  const auto& to = waypoint_road_ids_.back();
  const auto sub_path_opt = graph_.FindRoute(from, to);

  if (!sub_path_opt.has_value()) {
    route_error_ = "No path found between road " + from + " and " + to;
    active_route_ = std::nullopt;
    sub_routes_.push_back(Route{});
    return false;
  }

  route_error_.clear();
  sub_routes_.push_back(*sub_path_opt);

  auto merged = Route{};
  for (const auto& sub_route : sub_routes_) {
    if (sub_route.segments.empty()) {
      active_route_ = std::nullopt;
      return false;
    }
    for (const auto& seg : sub_route.segments) {
      if (merged.segments.empty() || merged.segments.back().road_id != seg.road_id) {
        merged.segments.push_back(seg);
      }
    }
  }

  active_route_ = merged;
  return true;
}

void RouteBuilder::Undo() {
  if (waypoint_road_ids_.empty()) {
    return;
  }

  waypoint_road_ids_.pop_back();
  if (!sub_routes_.empty()) {
    sub_routes_.pop_back();
  }

  route_error_.clear();
  if (waypoint_road_ids_.size() < 2) {
    active_route_ = std::nullopt;
    return;
  }

  auto merged = Route{};
  for (std::size_t i = 0; i < sub_routes_.size(); ++i) {
    const auto& sub_route = sub_routes_[i];
    if (sub_route.segments.empty()) {
      active_route_ = std::nullopt;
      route_error_ = "No path found between road " + waypoint_road_ids_[i] + " and " + waypoint_road_ids_[i + 1];
      return;
    }
    for (const auto& seg : sub_route.segments) {
      if (merged.segments.empty() || merged.segments.back().road_id != seg.road_id) {
        merged.segments.push_back(seg);
      }
    }
  }

  active_route_ = merged;
}

void RouteBuilder::Clear() {
  waypoint_road_ids_.clear();
  sub_routes_.clear();
  active_route_ = std::nullopt;
  route_error_.clear();
}

auto RouteBuilder::Waypoints() const -> const std::vector<std::string>& { return waypoint_road_ids_; }

auto RouteBuilder::ActiveRoute() const -> const std::optional<Route>& { return active_route_; }

auto RouteBuilder::RouteError() const -> std::string_view { return route_error_; }

}  // namespace strada::routing
