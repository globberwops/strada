#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <queue>
#include <strada/ast/junction.hpp>
#include <strada/ast/lanes.hpp>
#include <strada/ast/road.hpp>
#include <strada/routing/graph.hpp>
#include <unordered_map>

namespace strada::routing {

namespace {

auto IsDrivableType(ast::LaneType type) -> bool {
  return type == ast::LaneType::kDriving || type == ast::LaneType::kOnRamp || type == ast::LaneType::kExit ||
         type == ast::LaneType::kConnectingRamp || type == ast::LaneType::kEntry || type == ast::LaneType::kOffRamp ||
         type == ast::LaneType::kMwyEntry || type == ast::LaneType::kMwyExit || type == ast::LaneType::kSlipLane;
}

auto HasDrivableLane(const ast::Road& road, bool forward) -> bool {
  for (const auto& section : road.lanes.sections) {
    if (forward) {
      for (const auto& lane : section.right) {
        if (lane.id < 0 && IsDrivableType(lane.type)) {
          return true;
        }
      }
    } else {
      for (const auto& lane : section.left) {
        if (lane.id > 0 && IsDrivableType(lane.type)) {
          return true;
        }
      }
    }
  }
  return false;
}

}  // namespace

Graph::Graph(const ast::AbstractSyntaxTree& ast) {
  const auto num_roads = ast.roads.size();
  nodes_.resize(2 * num_roads);
  idx_to_road_id_.resize(num_roads);

  for (std::size_t i = 0; i < num_roads; ++i) {
    const auto& road = ast.roads[i];
    road_id_to_idx_[road.id] = i;
    idx_to_road_id_[i] = road.id;

    const auto is_junc = (road.junction != "-1" && !road.junction.empty());

    // Forward state
    const auto forward_idx = DirectedRoad{i, DirectedRoad::Direction::kForward}.ToNodeIndex();
    nodes_[forward_idx].road_id = road.id;
    nodes_[forward_idx].length = road.length;
    nodes_[forward_idx].is_junction = is_junc;
    nodes_[forward_idx].is_drivable = HasDrivableLane(road, true);

    // Backward state
    const auto backward_idx = DirectedRoad{i, DirectedRoad::Direction::kBackward}.ToNodeIndex();
    nodes_[backward_idx].road_id = road.id;
    nodes_[backward_idx].length = road.length;
    nodes_[backward_idx].is_junction = is_junc;
    nodes_[backward_idx].is_drivable = HasDrivableLane(road, false);
  }

  // 1. Direct Road Links
  for (std::size_t i = 0; i < num_roads; ++i) {
    const auto& road = ast.roads[i];
    const auto i_forward = DirectedRoad{i, DirectedRoad::Direction::kForward}.ToNodeIndex();
    const auto i_backward = DirectedRoad{i, DirectedRoad::Direction::kBackward}.ToNodeIndex();

    // Successor Link
    if (road.link.successor && road.link.successor->element_type == ast::RoadLinkType::kRoad) {
      auto target_it = road_id_to_idx_.find(road.link.successor->element_id);
      if (target_it != road_id_to_idx_.end()) {
        const auto j = target_it->second;
        const auto j_forward = DirectedRoad{j, DirectedRoad::Direction::kForward}.ToNodeIndex();
        const auto j_backward = DirectedRoad{j, DirectedRoad::Direction::kBackward}.ToNodeIndex();
        const auto contact_point = road.link.successor->contact_point.value_or(ast::ContactPoint::kStart);

        if (contact_point == ast::ContactPoint::kStart) {
          if (nodes_[i_forward].is_drivable && nodes_[j_forward].is_drivable) {
            nodes_[i_forward].successors.push_back(j_forward);
          }
          if (nodes_[i_backward].is_drivable && nodes_[j_backward].is_drivable) {
            nodes_[j_backward].successors.push_back(i_backward);
          }
        } else {  // cp == ContactPoint::kEnd
          if (nodes_[i_forward].is_drivable && nodes_[j_backward].is_drivable) {
            nodes_[i_forward].successors.push_back(j_backward);
          }
          if (nodes_[i_backward].is_drivable && nodes_[j_forward].is_drivable) {
            nodes_[j_forward].successors.push_back(i_backward);
          }
        }
      }
    }

    // Predecessor Link
    if (road.link.predecessor && road.link.predecessor->element_type == ast::RoadLinkType::kRoad) {
      auto target_it = road_id_to_idx_.find(road.link.predecessor->element_id);
      if (target_it != road_id_to_idx_.end()) {
        const auto j = target_it->second;
        const auto j_forward = DirectedRoad{j, DirectedRoad::Direction::kForward}.ToNodeIndex();
        const auto j_backward = DirectedRoad{j, DirectedRoad::Direction::kBackward}.ToNodeIndex();
        const auto contact_point = road.link.predecessor->contact_point.value_or(ast::ContactPoint::kStart);

        if (contact_point == ast::ContactPoint::kStart) {
          if (nodes_[i_backward].is_drivable && nodes_[j_forward].is_drivable) {
            nodes_[i_backward].successors.push_back(j_forward);
          }
          if (nodes_[i_forward].is_drivable && nodes_[j_backward].is_drivable) {
            nodes_[j_backward].successors.push_back(i_forward);
          }
        } else {  // cp == ContactPoint::kEnd
          if (nodes_[i_backward].is_drivable && nodes_[j_backward].is_drivable) {
            nodes_[i_backward].successors.push_back(j_backward);
          }
          if (nodes_[i_forward].is_drivable && nodes_[j_forward].is_drivable) {
            nodes_[j_forward].successors.push_back(i_forward);
          }
        }
      }
    }
  }

  // 2. Junction Connections
  for (const auto& junction : ast.junctions) {
    for (const auto& conn : junction.connections) {
      auto a_it = road_id_to_idx_.find(conn.incoming_road);
      auto c_it = road_id_to_idx_.find(conn.connecting_road);
      if (a_it == road_id_to_idx_.end() || c_it == road_id_to_idx_.end()) {
        continue;
      }
      const auto a = a_it->second;
      const auto c = c_it->second;
      const auto a_forward = DirectedRoad{a, DirectedRoad::Direction::kForward}.ToNodeIndex();
      const auto a_backward = DirectedRoad{a, DirectedRoad::Direction::kBackward}.ToNodeIndex();
      const auto c_forward = DirectedRoad{c, DirectedRoad::Direction::kForward}.ToNodeIndex();
      const auto c_backward = DirectedRoad{c, DirectedRoad::Direction::kBackward}.ToNodeIndex();

      auto incoming_is_successor = true;
      const ast::Road* road_a = nullptr;
      for (const auto& r : ast.roads) {
        if (r.id == conn.incoming_road) {
          road_a = &r;
          break;
        }
      }
      if (road_a != nullptr) {
        if (road_a->link.predecessor && road_a->link.predecessor->element_type == ast::RoadLinkType::kJunction &&
            road_a->link.predecessor->element_id == junction.id) {
          incoming_is_successor = false;
        }
      }

      {
        const auto contact_point = conn.contact_point;

        if (incoming_is_successor) {
          if (nodes_[a_forward].is_drivable) {
            if (contact_point == ast::ContactPoint::kStart && nodes_[c_forward].is_drivable) {
              nodes_[a_forward].successors.push_back(c_forward);
            } else if (contact_point == ast::ContactPoint::kEnd && nodes_[c_backward].is_drivable) {
              nodes_[a_forward].successors.push_back(c_backward);
            }
          }
          if (nodes_[a_backward].is_drivable) {
            if (contact_point == ast::ContactPoint::kStart && nodes_[c_backward].is_drivable) {
              nodes_[c_backward].successors.push_back(a_backward);
            } else if (contact_point == ast::ContactPoint::kEnd && nodes_[c_forward].is_drivable) {
              nodes_[c_forward].successors.push_back(a_backward);
            }
          }
        } else {
          if (nodes_[a_backward].is_drivable) {
            if (contact_point == ast::ContactPoint::kStart && nodes_[c_forward].is_drivable) {
              nodes_[a_backward].successors.push_back(c_forward);
            } else if (contact_point == ast::ContactPoint::kEnd && nodes_[c_backward].is_drivable) {
              nodes_[a_backward].successors.push_back(c_backward);
            }
          }
          if (nodes_[a_forward].is_drivable) {
            if (contact_point == ast::ContactPoint::kStart && nodes_[c_backward].is_drivable) {
              nodes_[c_backward].successors.push_back(a_forward);
            } else if (contact_point == ast::ContactPoint::kEnd && nodes_[c_forward].is_drivable) {
              nodes_[c_forward].successors.push_back(a_forward);
            }
          }
        }
      }
    }
  }

  // Deduplicate successors
  for (auto& node : nodes_) {
    std::ranges::sort(node.successors);
    auto ret = std::ranges::unique(node.successors);
    node.successors.erase(ret.begin(), ret.end());
  }
}

auto Graph::FindRoute(std::string_view start_road_id, std::string_view end_road_id) const -> std::optional<Route> {
  const auto start_it = road_id_to_idx_.find(start_road_id);
  const auto end_it = road_id_to_idx_.find(end_road_id);
  if (start_it == road_id_to_idx_.end() || end_it == road_id_to_idx_.end()) {
    return std::nullopt;
  }

  const auto start_idx = start_it->second;
  const auto end_idx = end_it->second;

  const auto num_states = nodes_.size();
  auto dist = std::vector<double>(num_states, std::numeric_limits<double>::infinity());
  auto parent = std::vector<std::size_t>(num_states, std::numeric_limits<std::size_t>::max());

  using StatePair = std::pair<double, std::size_t>;
  auto queue = std::priority_queue<StatePair, std::vector<StatePair>, std::greater<>>{};

  auto initialized = false;
  for (const auto dir : {DirectedRoad::Direction::kForward, DirectedRoad::Direction::kBackward}) {
    const auto state_idx = DirectedRoad{start_idx, dir}.ToNodeIndex();
    if (nodes_[state_idx].is_drivable) {
      const auto cost = nodes_[state_idx].length;
      dist[state_idx] = cost;
      queue.emplace(cost, state_idx);
      initialized = true;
    }
  }

  if (!initialized) {
    return std::nullopt;
  }

  while (!queue.empty()) {
    const auto [distance, state_idx] = queue.top();
    queue.pop();

    if (distance > dist[state_idx]) {
      continue;
    }

    if (DirectedRoad::FromNodeIndex(state_idx).road_idx == end_idx) {
      auto path_indices = std::vector<std::size_t>{};
      auto curr = state_idx;
      while (curr != std::numeric_limits<std::size_t>::max()) {
        path_indices.push_back(curr);
        curr = parent[curr];
      }
      std::ranges::reverse(path_indices);

      auto route = Route{};
      route.segments.reserve(path_indices.size());
      for (const auto idx : path_indices) {
        auto seg = RouteSegment{};
        seg.road_id = nodes_[idx].road_id;
        seg.forward = (DirectedRoad::FromNodeIndex(idx).direction == DirectedRoad::Direction::kForward);
        seg.length = nodes_[idx].length;
        route.segments.push_back(std::move(seg));
      }
      return route;
    }

    for (const auto successor_idx : nodes_[state_idx].successors) {
      const auto weight = nodes_[successor_idx].length;
      if (dist[state_idx] + weight < dist[successor_idx]) {
        dist[successor_idx] = dist[state_idx] + weight;
        parent[successor_idx] = state_idx;
        queue.emplace(dist[successor_idx], successor_idx);
      }
    }
  }

  return std::nullopt;
}

auto Graph::FindPath(std::string_view start_road_id, std::string_view end_road_id) const
    -> std::optional<std::vector<std::string>> {
  return FindPath(start_road_id, end_road_id,
                  [this](std::string_view road_id) -> double { return GetRoadLength(road_id); });
}

auto Graph::HasRoad(std::string_view road_id) const -> bool { return road_id_to_idx_.contains(road_id); }

auto Graph::GetRoadSuccessors(std::string_view road_id) const -> std::vector<std::string> {
  auto it = road_id_to_idx_.find(road_id);
  if (it == road_id_to_idx_.end()) {
    return {};
  }
  const auto idx = it->second;
  std::vector<std::string> successors;

  const auto forward_idx = DirectedRoad{idx, DirectedRoad::Direction::kForward}.ToNodeIndex();
  for (const auto v : nodes_[forward_idx].successors) {
    successors.push_back(nodes_[v].road_id);
  }
  const auto backward_idx = DirectedRoad{idx, DirectedRoad::Direction::kBackward}.ToNodeIndex();
  for (const auto v : nodes_[backward_idx].successors) {
    successors.push_back(nodes_[v].road_id);
  }

  std::ranges::sort(successors);
  auto ret = std::ranges::unique(successors);
  successors.erase(ret.begin(), ret.end());

  return successors;
}

auto Graph::IsJunctionRoad(std::string_view road_id) const -> bool {
  auto it = road_id_to_idx_.find(road_id);
  if (it == road_id_to_idx_.end()) {
    return false;
  }
  const auto forward_idx = DirectedRoad{it->second, DirectedRoad::Direction::kForward}.ToNodeIndex();
  return nodes_[forward_idx].is_junction;
}

auto Graph::GetRoadLength(std::string_view road_id) const -> double {
  auto it = road_id_to_idx_.find(road_id);
  if (it == road_id_to_idx_.end()) {
    return 0.0;
  }
  const auto forward_idx = DirectedRoad{it->second, DirectedRoad::Direction::kForward}.ToNodeIndex();
  return nodes_[forward_idx].length;
}

auto Graph::FindPathImpl(std::string_view start_road_id, std::string_view end_road_id,
                         const std::function<double(std::string_view)>& cost_fn) const
    -> std::optional<std::vector<std::string>> {
  if (start_road_id == end_road_id) {
    if (HasRoad(start_road_id)) {
      return std::vector<std::string>{std::string(start_road_id)};
    }
    return std::nullopt;
  }

  auto start_it = road_id_to_idx_.find(start_road_id);
  auto end_it = road_id_to_idx_.find(end_road_id);
  if (start_it == road_id_to_idx_.end() || end_it == road_id_to_idx_.end()) {
    return std::nullopt;
  }

  const auto start_idx = start_it->second;
  const auto end_idx = end_it->second;

  const auto num_states = nodes_.size();
  auto dist = std::vector(num_states, std::numeric_limits<double>::infinity());
  auto parent = std::vector(num_states, std::numeric_limits<std::size_t>::max());

  using StatePair = std::pair<double, std::size_t>;
  std::priority_queue<StatePair, std::vector<StatePair>, std::greater<>> queue;

  // Initialize start states
  auto initialized = false;
  for (const auto dir : {DirectedRoad::Direction::kForward, DirectedRoad::Direction::kBackward}) {
    const auto state_idx = DirectedRoad{start_idx, dir}.ToNodeIndex();
    if (nodes_[state_idx].is_drivable) {
      const auto cost = cost_fn(start_road_id);
      dist[state_idx] = cost;
      queue.emplace(cost, state_idx);
      initialized = true;
    }
  }

  if (!initialized) {
    return std::nullopt;
  }

  while (!queue.empty()) {
    auto [d, u] = queue.top();
    queue.pop();

    if (d > dist[u]) {
      continue;
    }

    if (DirectedRoad::FromNodeIndex(u).road_idx == end_idx) {
      std::vector<std::string> path;
      std::size_t curr = u;
      while (curr != std::numeric_limits<std::size_t>::max()) {
        path.push_back(nodes_[curr].road_id);
        curr = parent[curr];
      }
      std::ranges::reverse(path);
      return path;
    }

    for (const auto v : nodes_[u].successors) {
      double weight = cost_fn(nodes_[v].road_id);
      if (dist[u] + weight < dist[v]) {
        dist[v] = dist[u] + weight;
        parent[v] = u;
        queue.emplace(dist[v], v);
      }
    }
  }

  return std::nullopt;
}

auto Route::ToRouteCoordinates(std::string_view road_id, double s_local, double t_local,
                               std::optional<double> s_route_hint) const noexcept
    -> std::optional<std::pair<double, double>> {
  auto start_s = 0.0;
  std::optional<std::pair<double, double>> best_match;
  auto min_diff = std::numeric_limits<double>::infinity();

  for (const auto& seg : segments) {
    if (seg.road_id == road_id) {
      auto s_route = 0.0;
      auto t_route = 0.0;
      if (seg.forward) {
        s_route = start_s + s_local;
        t_route = t_local;
      } else {
        s_route = start_s + (seg.length - s_local);
        t_route = -t_local;
      }
      const auto candidate = std::make_pair(s_route, t_route);

      if (!s_route_hint.has_value()) {
        return candidate;
      }

      const auto diff = std::abs(s_route - *s_route_hint);
      if (diff < min_diff) {
        min_diff = diff;
        best_match = candidate;
      }
    }
    start_s += seg.length;
  }
  return best_match;
}

}  // namespace strada::routing
