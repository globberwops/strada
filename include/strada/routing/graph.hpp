#pragma once

#include <concepts>
#include <functional>
#include <optional>
#include <strada/ast/abstract_syntax_tree.hpp>
#include <string>
#include <unordered_map>
#include <vector>

namespace strada::routing {

/// Concept that constrains the cost functor to be callable with a road ID and return a value convertible to double.
template <typename F>
concept CostFunction = requires(F&& f, std::string_view road_id) {
  { f(road_id) } -> std::convertible_to<double>;
};

/// Represents a single segment along a Route.
struct RouteSegment {
  std::string road_id;
  bool forward{true};
  double length{0.0};
};

/// Represents a planned route consisting of multiple segments.
struct Route {
  std::vector<RouteSegment> segments;
};

/// Represents the road-level topological graph of the OpenDRIVE map.
class Graph {
 public:
  /// Constructs the routing graph from the parsed Abstract Syntax Tree.
  /// Does not store any reference to the AST inside.
  explicit Graph(const ast::AbstractSyntaxTree& ast);

  /// Finds the shortest route between start_road_id and end_road_id resolving segment directions.
  ///
  /// \param start_road_id The ID of the starting road.
  /// \param end_road_id The ID of the destination road.
  /// \return The calculated Route, or std::nullopt if no drivable path exists.
  auto FindRoute(std::string_view start_road_id, std::string_view end_road_id) const -> std::optional<Route>;

  /// Finds the shortest path between start_road_id and end_road_id using the road lengths as weights.
  auto FindPath(std::string_view start_road_id, std::string_view end_road_id) const
      -> std::optional<std::vector<std::string>>;

  /// Finds the shortest path between start_road_id and end_road_id using a custom cost functor.
  template <CostFunction CostFn>
  auto FindPath(std::string_view start_road_id, std::string_view end_road_id, CostFn&& cost_fn) const
      -> std::optional<std::vector<std::string>> {
    return FindPathImpl(start_road_id, end_road_id,
                        std::function<double(std::string_view)>(std::forward<CostFn>(cost_fn)));
  }

  /// Returns true if the road exists in the graph.
  auto HasRoad(std::string_view road_id) const -> bool;

  /// Returns all unique successor roads that can be transitioned to from this road.
  auto GetRoadSuccessors(std::string_view road_id) const -> std::vector<std::string>;

  /// Returns true if the road is a connecting road within a junction.
  auto IsJunctionRoad(std::string_view road_id) const -> bool;

  /// Returns the length of the road.
  auto GetRoadLength(std::string_view road_id) const -> double;

 private:
  auto FindPathImpl(std::string_view start_road_id, std::string_view end_road_id,
                    const std::function<double(std::string_view)>& cost_fn) const
      -> std::optional<std::vector<std::string>>;

  struct Node {
    std::string road_id;
    double length{0.0};
    bool is_junction{false};
    bool is_drivable{false};
    std::vector<std::uint32_t> successors;  // Indices in nodes_ array (directed states)
  };

  struct StringHash {
    using is_transparent = void;
    auto operator()(std::string_view sv) const -> std::size_t { return std::hash<std::string_view>{}(sv); }
    auto operator()(const std::string& s) const -> std::size_t { return std::hash<std::string>{}(s); }
  };

  std::vector<Node> nodes_;  // Size: 2 * num_roads. Even: forward, Odd: backward.
  std::unordered_map<std::string, std::uint32_t, StringHash, std::equal_to<>>
      road_id_to_idx_;                       // Maps road_id to road index (0 to num_roads - 1)
  std::vector<std::string> idx_to_road_id_;  // Maps road index to road_id
};

}  // namespace strada::routing
