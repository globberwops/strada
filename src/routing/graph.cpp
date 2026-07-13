#include <algorithm>
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
         type == ast::LaneType::kConnectingRamp || type == ast::LaneType::kEntry;
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
  std::size_t num_roads = ast.roads.size();
  nodes_.resize(2 * num_roads);
  idx_to_road_id_.resize(num_roads);

  for (std::size_t i = 0; i < num_roads; ++i) {
    const auto& road = ast.roads[i];
    road_id_to_idx_[road.id] = static_cast<std::uint32_t>(i);
    idx_to_road_id_[i] = road.id;

    bool is_junc = (road.junction != "-1" && !road.junction.empty());

    // Forward state
    nodes_[2 * i].road_id = road.id;
    nodes_[2 * i].length = road.length;
    nodes_[2 * i].is_junction = is_junc;
    nodes_[2 * i].is_drivable = HasDrivableLane(road, true);

    // Backward state
    nodes_[2 * i + 1].road_id = road.id;
    nodes_[2 * i + 1].length = road.length;
    nodes_[2 * i + 1].is_junction = is_junc;
    nodes_[2 * i + 1].is_drivable = HasDrivableLane(road, false);
  }

  // 1. Direct Road Links
  for (std::size_t i = 0; i < num_roads; ++i) {
    const auto& road = ast.roads[i];

    // Successor Link
    if (road.link.successor && road.link.successor->element_type == ast::RoadLinkType::kRoad) {
      auto target_it = road_id_to_idx_.find(road.link.successor->element_id);
      if (target_it != road_id_to_idx_.end()) {
        std::uint32_t j = target_it->second;
        ast::ContactPoint cp = road.link.successor->contact_point.value_or(ast::ContactPoint::kStart);

        if (cp == ast::ContactPoint::kStart) {
          if (nodes_[2 * i].is_drivable && nodes_[2 * j].is_drivable) {
            nodes_[2 * i].successors.push_back(2 * j);
          }
          if (nodes_[2 * i + 1].is_drivable && nodes_[2 * j + 1].is_drivable) {
            nodes_[2 * j + 1].successors.push_back(static_cast<std::uint32_t>(2 * i + 1));
          }
        } else {  // cp == ContactPoint::kEnd
          if (nodes_[2 * i].is_drivable && nodes_[2 * j + 1].is_drivable) {
            nodes_[2 * i].successors.push_back(2 * j + 1);
          }
          if (nodes_[2 * i + 1].is_drivable && nodes_[2 * j].is_drivable) {
            nodes_[2 * j].successors.push_back(static_cast<std::uint32_t>(2 * i + 1));
          }
        }
      }
    }

    // Predecessor Link
    if (road.link.predecessor && road.link.predecessor->element_type == ast::RoadLinkType::kRoad) {
      auto target_it = road_id_to_idx_.find(road.link.predecessor->element_id);
      if (target_it != road_id_to_idx_.end()) {
        std::uint32_t j = target_it->second;
        ast::ContactPoint cp = road.link.predecessor->contact_point.value_or(ast::ContactPoint::kStart);

        if (cp == ast::ContactPoint::kStart) {
          if (nodes_[2 * i + 1].is_drivable && nodes_[2 * j].is_drivable) {
            nodes_[2 * i + 1].successors.push_back(2 * j);
          }
          if (nodes_[2 * i].is_drivable && nodes_[2 * j + 1].is_drivable) {
            nodes_[2 * j + 1].successors.push_back(static_cast<std::uint32_t>(2 * i));
          }
        } else {  // cp == ContactPoint::kEnd
          if (nodes_[2 * i + 1].is_drivable && nodes_[2 * j + 1].is_drivable) {
            nodes_[2 * i + 1].successors.push_back(2 * j + 1);
          }
          if (nodes_[2 * i].is_drivable && nodes_[2 * j].is_drivable) {
            nodes_[2 * j].successors.push_back(static_cast<std::uint32_t>(2 * i));
          }
        }
      }
    }
  }

  // 2. Junction Connections
  for (const auto& junction : ast.junctions) {
    for (const auto& conn : junction.connections) {
      auto A_it = road_id_to_idx_.find(conn.incoming_road);
      auto C_it = road_id_to_idx_.find(conn.connecting_road);
      if (A_it == road_id_to_idx_.end() || C_it == road_id_to_idx_.end()) {
        continue;
      }
      std::uint32_t a = A_it->second;
      std::uint32_t c = C_it->second;

      bool incoming_is_successor = true;
      const ast::Road* road_A = nullptr;
      for (const auto& r : ast.roads) {
        if (r.id == conn.incoming_road) {
          road_A = &r;
          break;
        }
      }
      if (road_A) {
        if (road_A->link.predecessor && road_A->link.predecessor->element_type == ast::RoadLinkType::kJunction &&
            road_A->link.predecessor->element_id == junction.id) {
          incoming_is_successor = false;
        }
      }

      {
        ast::ContactPoint cp = conn.contact_point;

        if (incoming_is_successor) {
          if (nodes_[2 * a].is_drivable) {
            if (cp == ast::ContactPoint::kStart && nodes_[2 * c].is_drivable) {
              nodes_[2 * a].successors.push_back(2 * c);
            } else if (cp == ast::ContactPoint::kEnd && nodes_[2 * c + 1].is_drivable) {
              nodes_[2 * a].successors.push_back(2 * c + 1);
            }
          }
          if (nodes_[2 * a + 1].is_drivable) {
            if (cp == ast::ContactPoint::kStart && nodes_[2 * c + 1].is_drivable) {
              nodes_[2 * c + 1].successors.push_back(2 * a + 1);
            } else if (cp == ast::ContactPoint::kEnd && nodes_[2 * c].is_drivable) {
              nodes_[2 * c].successors.push_back(2 * a + 1);
            }
          }
        } else {
          if (nodes_[2 * a + 1].is_drivable) {
            if (cp == ast::ContactPoint::kStart && nodes_[2 * c].is_drivable) {
              nodes_[2 * a + 1].successors.push_back(2 * c);
            } else if (cp == ast::ContactPoint::kEnd && nodes_[2 * c + 1].is_drivable) {
              nodes_[2 * a + 1].successors.push_back(2 * c + 1);
            }
          }
          if (nodes_[2 * a].is_drivable) {
            if (cp == ast::ContactPoint::kStart && nodes_[2 * c + 1].is_drivable) {
              nodes_[2 * c + 1].successors.push_back(2 * a);
            } else if (cp == ast::ContactPoint::kEnd && nodes_[2 * c].is_drivable) {
              nodes_[2 * c].successors.push_back(2 * a);
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
  auto start_it = road_id_to_idx_.find(start_road_id);
  auto end_it = road_id_to_idx_.find(end_road_id);
  if (start_it == road_id_to_idx_.end() || end_it == road_id_to_idx_.end()) {
    return std::nullopt;
  }

  std::uint32_t start_idx = start_it->second;
  std::uint32_t end_idx = end_it->second;

  std::size_t num_states = nodes_.size();
  std::vector<double> dist(num_states, std::numeric_limits<double>::infinity());
  std::vector<std::uint32_t> parent(num_states, std::numeric_limits<std::uint32_t>::max());

  using StatePair = std::pair<double, std::uint32_t>;
  std::priority_queue<StatePair, std::vector<StatePair>, std::greater<StatePair>> pq;

  bool initialized = false;
  if (nodes_[2 * start_idx].is_drivable) {
    double cost = nodes_[2 * start_idx].length;
    dist[2 * start_idx] = cost;
    pq.push({cost, 2 * start_idx});
    initialized = true;
  }
  if (nodes_[2 * start_idx + 1].is_drivable) {
    double cost = nodes_[2 * start_idx + 1].length;
    dist[2 * start_idx + 1] = cost;
    pq.push({cost, 2 * start_idx + 1});
    initialized = true;
  }

  if (!initialized) {
    return std::nullopt;
  }

  while (!pq.empty()) {
    auto [d, u] = pq.top();
    pq.pop();

    if (d > dist[u]) {
      continue;
    }

    if (u / 2 == end_idx) {
      std::vector<std::uint32_t> path_indices;
      std::uint32_t curr = u;
      while (curr != std::numeric_limits<std::uint32_t>::max()) {
        path_indices.push_back(curr);
        curr = parent[curr];
      }
      std::reverse(path_indices.begin(), path_indices.end());

      Route route;
      route.segments.reserve(path_indices.size());
      for (std::uint32_t idx : path_indices) {
        RouteSegment seg;
        seg.road_id = nodes_[idx].road_id;
        seg.forward = (idx % 2 == 0);
        seg.length = nodes_[idx].length;
        route.segments.push_back(std::move(seg));
      }
      return route;
    }

    for (std::uint32_t v : nodes_[u].successors) {
      double weight = nodes_[v].length;
      if (dist[u] + weight < dist[v]) {
        dist[v] = dist[u] + weight;
        parent[v] = u;
        pq.push({dist[v], v});
      }
    }
  }

  return std::nullopt;
}

auto Graph::FindPath(std::string_view start_road_id, std::string_view end_road_id) const
    -> std::optional<std::vector<std::string>> {
  return FindPath(start_road_id, end_road_id, [this](std::string_view road_id) { return GetRoadLength(road_id); });
}

auto Graph::HasRoad(std::string_view road_id) const -> bool { return road_id_to_idx_.contains(road_id); }

auto Graph::GetRoadSuccessors(std::string_view road_id) const -> std::vector<std::string> {
  auto it = road_id_to_idx_.find(road_id);
  if (it == road_id_to_idx_.end()) {
    return {};
  }
  std::size_t idx = it->second;
  std::vector<std::string> successors;

  for (std::uint32_t v : nodes_[2 * idx].successors) {
    successors.push_back(nodes_[v].road_id);
  }
  for (std::uint32_t v : nodes_[2 * idx + 1].successors) {
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
  return nodes_[2 * it->second].is_junction;
}

auto Graph::GetRoadLength(std::string_view road_id) const -> double {
  auto it = road_id_to_idx_.find(road_id);
  if (it == road_id_to_idx_.end()) {
    return 0.0;
  }
  return nodes_[2 * it->second].length;
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

  std::uint32_t start_idx = start_it->second;
  std::uint32_t end_idx = end_it->second;

  std::size_t num_states = nodes_.size();
  std::vector<double> dist(num_states, std::numeric_limits<double>::infinity());
  std::vector<std::uint32_t> parent(num_states, std::numeric_limits<std::uint32_t>::max());

  using StatePair = std::pair<double, std::uint32_t>;
  std::priority_queue<StatePair, std::vector<StatePair>, std::greater<StatePair>> pq;

  // Initialize start states
  bool initialized = false;
  if (nodes_[2 * start_idx].is_drivable) {
    double cost = cost_fn(start_road_id);
    dist[2 * start_idx] = cost;
    pq.push({cost, 2 * start_idx});
    initialized = true;
  }
  if (nodes_[2 * start_idx + 1].is_drivable) {
    double cost = cost_fn(start_road_id);
    dist[2 * start_idx + 1] = cost;
    pq.push({cost, 2 * start_idx + 1});
    initialized = true;
  }

  if (!initialized) {
    return std::nullopt;
  }

  while (!pq.empty()) {
    auto [d, u] = pq.top();
    pq.pop();

    if (d > dist[u]) {
      continue;
    }

    if (u / 2 == end_idx) {
      std::vector<std::string> path;
      std::uint32_t curr = u;
      while (curr != std::numeric_limits<std::uint32_t>::max()) {
        path.push_back(nodes_[curr].road_id);
        curr = parent[curr];
      }
      std::reverse(path.begin(), path.end());
      return path;
    }

    for (std::uint32_t v : nodes_[u].successors) {
      double weight = cost_fn(nodes_[v].road_id);
      if (dist[u] + weight < dist[v]) {
        dist[v] = dist[u] + weight;
        parent[v] = u;
        pq.push({dist[v], v});
      }
    }
  }

  return std::nullopt;
}

}  // namespace strada::routing
