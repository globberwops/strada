// This file implements the RouteBuilder utility class.

#include "strada/routing/route_builder.hpp"

#include <optional>
#include <strada/routing/graph.hpp>
#include <string>
#include <string_view>
#include <vector>

namespace strada::routing {

RouteBuilder::RouteBuilder(const Graph* graph) : graph_(graph) {}

auto RouteBuilder::AppendWaypoint(std::string_view road_id) -> bool {
  if (graph_ == nullptr) {
    route_error_ = "Graph is not initialized.";
    return false;
  }

  waypoint_road_ids_.emplace_back(road_id);

  if (waypoint_road_ids_.size() < 2) {
    active_route_ = std::nullopt;
    route_error_.clear();
    route_history_.push_back(active_route_);
    error_history_.push_back(route_error_);
    return true;
  }

  const auto& from_id = waypoint_road_ids_[waypoint_road_ids_.size() - 2];
  const auto& to_id = waypoint_road_ids_.back();
  const auto sub_path_opt = graph_->FindRoute(from_id, to_id);

  if (!sub_path_opt.has_value()) {
    route_error_ = "No path found between road " + from_id + " and " + to_id;
    active_route_ = std::nullopt;
    sub_routes_.emplace_back();
    route_history_.push_back(active_route_);
    error_history_.push_back(route_error_);
    return false;
  }

  route_error_.clear();
  sub_routes_.push_back(*sub_path_opt);

  auto merged = Route{};
  for (std::size_t i = 0; i < sub_routes_.size(); ++i) {
    if (sub_routes_[i].segments.empty()) {
      route_error_ = "No path found between road " + waypoint_road_ids_[i] + " and " + waypoint_road_ids_[i + 1];
      active_route_ = std::nullopt;
      route_history_.push_back(active_route_);
      error_history_.push_back(route_error_);
      return false;
    }
    for (const auto& seg : sub_routes_[i].segments) {
      if (merged.segments.empty() || merged.segments.back().road_id != seg.road_id) {
        merged.segments.push_back(seg);
      }
    }
  }

  active_route_ = merged;
  route_history_.push_back(active_route_);
  error_history_.push_back(route_error_);
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

  if (!route_history_.empty()) {
    route_history_.pop_back();
  }
  if (!error_history_.empty()) {
    error_history_.pop_back();
  }

  if (route_history_.empty()) {
    active_route_ = std::nullopt;
    route_error_.clear();
  } else {
    active_route_ = route_history_.back();
    route_error_ = error_history_.back();
  }
}

void RouteBuilder::Clear() {
  waypoint_road_ids_.clear();
  sub_routes_.clear();
  route_history_.clear();
  error_history_.clear();
  active_route_ = std::nullopt;
  route_error_.clear();
}

auto RouteBuilder::Waypoints() const -> const std::vector<std::string>& { return waypoint_road_ids_; }

auto RouteBuilder::ActiveRoute() const -> const std::optional<Route>& { return active_route_; }

auto RouteBuilder::RouteError() const -> std::string_view { return route_error_; }

}  // namespace strada::routing
