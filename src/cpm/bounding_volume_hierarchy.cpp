#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <strada/cpm/bounding_volume_hierarchy.hpp>

namespace strada::cpm {

namespace {

struct BoundingVolumeHierarchyAabb {
  double min_x{std::numeric_limits<double>::max()};
  double min_y{std::numeric_limits<double>::max()};
  double max_x{-std::numeric_limits<double>::max()};
  double max_y{-std::numeric_limits<double>::max()};

  void Grow(const BoundingVolumeHierarchyAabb& other) noexcept {
    min_x = std::min(min_x, other.min_x);
    min_y = std::min(min_y, other.min_y);
    max_x = std::max(max_x, other.max_x);
    max_y = std::max(max_y, other.max_y);
  }

  void Grow(const Aabb& other) noexcept {
    min_x = std::min(min_x, other.min_x);
    min_y = std::min(min_y, other.min_y);
    max_x = std::max(max_x, other.max_x);
    max_y = std::max(max_y, other.max_y);
  }

  void Grow(double px, double py) noexcept {
    min_x = std::min(min_x, px);
    min_y = std::min(min_y, py);
    max_x = std::max(max_x, px);
    max_y = std::max(max_y, py);
  }

  [[nodiscard]] auto Area() const noexcept -> double {
    double dx = max_x - min_x;
    double dy = max_y - min_y;
    return (dx > 0.0 ? dx : 0.0) + (dy > 0.0 ? dy : 0.0);
  }
};

auto MakeLeafNode(std::vector<BoundingVolumeHierarchy::Node>& nodes, uint32_t node_idx,
                  const BoundingVolumeHierarchyAabb& bounds,
                  std::vector<BoundingVolumeHierarchy::PrimitiveInfo>& final_primitives,
                  const std::vector<uint32_t>& prim_indices,
                  const std::vector<BoundingVolumeHierarchy::PrimitiveInfo>& temp_primitives, uint32_t start_idx,
                  uint32_t count) noexcept -> uint32_t {
  auto prim_start = static_cast<uint32_t>(final_primitives.size());
  for (uint32_t idx = 0; idx < count; ++idx) {
    final_primitives.push_back(temp_primitives[prim_indices[start_idx + idx]]);
  }

  nodes[node_idx].min_x = bounds.min_x;
  nodes[node_idx].min_y = bounds.min_y;
  nodes[node_idx].max_x = bounds.max_x;
  nodes[node_idx].max_y = bounds.max_y;
  nodes[node_idx].left = prim_start;
  nodes[node_idx].right = count | 0x80000000;

  return node_idx;
}

auto BuildBoundingVolumeHierarchyRecursive(std::vector<BoundingVolumeHierarchy::Node>& nodes,
                                           std::vector<BoundingVolumeHierarchy::PrimitiveInfo>& final_primitives,
                                           std::vector<uint32_t>& prim_indices,
                                           const std::vector<BoundingVolumeHierarchy::PrimitiveInfo>& temp_primitives,
                                           const std::vector<Aabb>& temp_aabbs, uint32_t start_idx,
                                           uint32_t end_idx) noexcept -> uint32_t {
  auto node_idx = static_cast<uint32_t>(nodes.size());
  nodes.push_back(BoundingVolumeHierarchy::Node{});

  BoundingVolumeHierarchyAabb bounds;
  BoundingVolumeHierarchyAabb centroid_bounds;
  for (uint32_t idx = start_idx; idx < end_idx; ++idx) {
    uint32_t prim_idx = prim_indices[idx];
    bounds.Grow(temp_aabbs[prim_idx]);
    double cx = 0.5 * (temp_aabbs[prim_idx].min_x + temp_aabbs[prim_idx].max_x);
    double cy = 0.5 * (temp_aabbs[prim_idx].min_y + temp_aabbs[prim_idx].max_y);
    centroid_bounds.Grow(cx, cy);
  }

  uint32_t count = end_idx - start_idx;
  constexpr uint32_t kLeafThreshold = 4;

  if (count <= kLeafThreshold) {
    return MakeLeafNode(nodes, node_idx, bounds, final_primitives, prim_indices, temp_primitives, start_idx, count);
  }

  double ext_x = centroid_bounds.max_x - centroid_bounds.min_x;
  double ext_y = centroid_bounds.max_y - centroid_bounds.min_y;
  int axis = (ext_x > ext_y) ? 0 : 1;

  double min_coord = (axis == 0) ? centroid_bounds.min_x : centroid_bounds.min_y;
  double max_coord = (axis == 0) ? centroid_bounds.max_x : centroid_bounds.max_y;

  if (max_coord - min_coord < 1e-9) {
    return MakeLeafNode(nodes, node_idx, bounds, final_primitives, prim_indices, temp_primitives, start_idx, count);
  }

  constexpr int kNumBins = 16;
  struct Bin {
    uint32_t count{0};
    BoundingVolumeHierarchyAabb bounds;
  };
  std::array<Bin, kNumBins> bins{};

  double scale = kNumBins / (max_coord - min_coord);
  for (uint32_t idx = start_idx; idx < end_idx; ++idx) {
    uint32_t prim_idx = prim_indices[idx];
    double centroid = (axis == 0) ? (0.5 * (temp_aabbs[prim_idx].min_x + temp_aabbs[prim_idx].max_x))
                                  : (0.5 * (temp_aabbs[prim_idx].min_y + temp_aabbs[prim_idx].max_y));
    int bin_idx = static_cast<int>((centroid - min_coord) * scale);
    bin_idx = std::clamp(bin_idx, 0, kNumBins - 1);
    bins[bin_idx].count++;
    bins[bin_idx].bounds.Grow(temp_aabbs[prim_idx]);
  }

  double min_split_cost = std::numeric_limits<double>::max();
  int best_split_bin = -1;

  std::array<BoundingVolumeHierarchyAabb, kNumBins - 1> left_bounds{};
  std::array<uint32_t, kNumBins - 1> left_counts{};
  BoundingVolumeHierarchyAabb left_accum;
  uint32_t left_cnt = 0;
  for (int idx = 0; idx < kNumBins - 1; ++idx) {
    left_accum.Grow(bins[idx].bounds);
    left_cnt += bins[idx].count;
    left_bounds[idx] = left_accum;
    left_counts[idx] = left_cnt;
  }

  std::array<BoundingVolumeHierarchyAabb, kNumBins - 1> right_bounds{};
  std::array<uint32_t, kNumBins - 1> right_counts{};
  BoundingVolumeHierarchyAabb right_accum;
  uint32_t right_cnt = 0;
  for (int idx = kNumBins - 1; idx > 0; --idx) {
    right_accum.Grow(bins[idx].bounds);
    right_cnt += bins[idx].count;
    right_bounds[idx - 1] = right_accum;
    right_counts[idx - 1] = right_cnt;
  }

  double parent_area = bounds.Area();
  constexpr double kCTrav = 1.0;
  constexpr double kCIsect = 1.0;

  for (int idx = 0; idx < kNumBins - 1; ++idx) {
    if (left_counts[idx] == 0 || right_counts[idx] == 0) {
      continue;
    }
    double cost =
        kCTrav +
        (kCIsect * (left_bounds[idx].Area() * left_counts[idx] + right_bounds[idx].Area() * right_counts[idx]) /
         parent_area);
    if (cost < min_split_cost) {
      min_split_cost = cost;
      best_split_bin = idx;
    }
  }

  double no_split_cost = count * kCIsect;

  if (min_split_cost >= no_split_cost) {
    return MakeLeafNode(nodes, node_idx, bounds, final_primitives, prim_indices, temp_primitives, start_idx, count);
  }

  auto split_it = std::stable_partition(
      prim_indices.begin() + start_idx, prim_indices.begin() + end_idx, [&](uint32_t prim_idx) -> bool {
        double centroid = (axis == 0) ? (0.5 * (temp_aabbs[prim_idx].min_x + temp_aabbs[prim_idx].max_x))
                                      : (0.5 * (temp_aabbs[prim_idx].min_y + temp_aabbs[prim_idx].max_y));
        int bin_idx = static_cast<int>((centroid - min_coord) * scale);
        bin_idx = std::clamp(bin_idx, 0, kNumBins - 1);
        return bin_idx <= best_split_bin;
      });

  auto mid_idx = static_cast<uint32_t>(std::distance(prim_indices.begin(), split_it));

  if (mid_idx == start_idx || mid_idx == end_idx) {
    mid_idx = start_idx + (count / 2);
  }

  uint32_t left_child = BuildBoundingVolumeHierarchyRecursive(nodes, final_primitives, prim_indices, temp_primitives,
                                                              temp_aabbs, start_idx, mid_idx);
  uint32_t right_child = BuildBoundingVolumeHierarchyRecursive(nodes, final_primitives, prim_indices, temp_primitives,
                                                               temp_aabbs, mid_idx, end_idx);

  nodes[node_idx].min_x = bounds.min_x;
  nodes[node_idx].min_y = bounds.min_y;
  nodes[node_idx].max_x = bounds.max_x;
  nodes[node_idx].max_y = bounds.max_y;
  nodes[node_idx].left = left_child;
  nodes[node_idx].right = right_child;

  return node_idx;
}

}  // namespace

auto BoundingVolumeHierarchy::Build(std::vector<uint32_t>& prim_indices,
                                    const std::vector<PrimitiveInfo>& temp_primitives,
                                    const std::vector<Aabb>& temp_aabbs) -> BoundingVolumeHierarchy {
  BoundingVolumeHierarchy bounding_volume_hierarchy;
  if (temp_primitives.empty()) {
    return bounding_volume_hierarchy;
  }
  BuildBoundingVolumeHierarchyRecursive(bounding_volume_hierarchy.nodes_, bounding_volume_hierarchy.primitives_,
                                        prim_indices, temp_primitives, temp_aabbs, 0,
                                        static_cast<uint32_t>(temp_primitives.size()));
  return bounding_volume_hierarchy;
}

auto BoundingVolumeHierarchy::DistancePointToAabb(double px, double py, double min_x, double min_y, double max_x,
                                                  double max_y) noexcept -> double {
  double dx = std::max({0.0, min_x - px, px - max_x});
  double dy = std::max({0.0, min_y - py, py - max_y});
  return std::sqrt((dx * dx) + (dy * dy));
}

}  // namespace strada::cpm
