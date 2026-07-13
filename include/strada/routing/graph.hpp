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
concept CostFunction = requires(F&& f, const std::string& road_id) {
  { f(road_id) } -> std::convertible_to<double>;
};

/// Represents the road-level topological graph of the OpenDRIVE map.
class Graph {
 public:
  /// Constructs the routing graph from the parsed Abstract Syntax Tree.
  /// Does not store any reference to the AST inside.
  explicit Graph(const ast::AbstractSyntaxTree& ast);

  /// Finds the shortest path between start_road_id and end_road_id using the road lengths as weights.
  auto FindPath(const std::string& start_road_id, const std::string& end_road_id) const
      -> std::optional<std::vector<std::string>>;

  /// Finds the shortest path between start_road_id and end_road_id using a custom cost functor.
  template <CostFunction CostFn>
  auto FindPath(const std::string& start_road_id, const std::string& end_road_id, CostFn&& cost_fn) const
      -> std::optional<std::vector<std::string>> {
    return FindPathImpl(start_road_id, end_road_id,
                        std::function<double(const std::string&)>(std::forward<CostFn>(cost_fn)));
  }

  /// Returns true if the road exists in the graph.
  auto HasRoad(const std::string& road_id) const -> bool;

  /// Returns all unique successor roads that can be transitioned to from this road.
  auto GetRoadSuccessors(const std::string& road_id) const -> std::vector<std::string>;

  /// Returns true if the road is a connecting road within a junction.
  auto IsJunctionRoad(const std::string& road_id) const -> bool;

  /// Returns the length of the road.
  auto GetRoadLength(const std::string& road_id) const -> double;

 private:
  auto FindPathImpl(const std::string& start_road_id, const std::string& end_road_id,
                    const std::function<double(const std::string&)>& cost_fn) const
      -> std::optional<std::vector<std::string>>;

  struct Node {
    std::string road_id;
    double length{0.0};
    bool is_junction{false};
    bool is_drivable{false};
    std::vector<uint32_t> successors;  // Indices in nodes_ array (directed states)
  };

  std::vector<Node> nodes_;                                   // Size: 2 * num_roads. Even: forward, Odd: backward.
  std::unordered_map<std::string, uint32_t> road_id_to_idx_;  // Maps road_id to road index (0 to num_roads - 1)
  std::vector<std::string> idx_to_road_id_;                   // Maps road index to road_id
};

}  // namespace strada::routing
