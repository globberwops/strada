#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <span>
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
    const double kDx = max_x - min_x;
    const double kDy = max_y - min_y;
    return (kDx > 0.0 ? kDx : 0.0) + (kDy > 0.0 ? kDy : 0.0);
  }
};

auto MakeLeafNode(std::vector<BoundingVolumeHierarchy::Node>& nodes, std::uint32_t node_idx,
                  const BoundingVolumeHierarchyAabb& bounds,
                  std::vector<BoundingVolumeHierarchy::PrimitiveInfo>& final_primitives,
                  const std::vector<std::uint32_t>& prim_indices,
                  std::span<const BoundingVolumeHierarchy::PrimitiveInfo> temp_primitives, std::uint32_t start_idx,
                  std::uint32_t count) noexcept -> std::uint32_t {
  auto prim_start = static_cast<std::uint32_t>(final_primitives.size());
  for (std::uint32_t idx = 0; idx < count; ++idx) {
    final_primitives.push_back(temp_primitives[prim_indices[start_idx + idx]]);
  }

  nodes[node_idx].min_x = bounds.min_x;
  nodes[node_idx].min_y = bounds.min_y;
  nodes[node_idx].max_x = bounds.max_x;
  nodes[node_idx].max_y = bounds.max_y;
  nodes[node_idx].left = prim_start;
  nodes[node_idx].right = count | BoundingVolumeHierarchy::kLeafBitMask;

  return node_idx;
}

auto BuildBoundingVolumeHierarchyRecursive(std::vector<BoundingVolumeHierarchy::Node>& nodes,
                                           std::vector<BoundingVolumeHierarchy::PrimitiveInfo>& final_primitives,
                                           std::vector<std::uint32_t>& prim_indices,
                                           std::span<const BoundingVolumeHierarchy::PrimitiveInfo> temp_primitives,
                                           std::span<const Aabb> temp_aabbs, std::uint32_t start_idx,
                                           std::uint32_t end_idx) noexcept -> std::uint32_t {
  auto node_idx = static_cast<std::uint32_t>(nodes.size());
  nodes.push_back(BoundingVolumeHierarchy::Node{});

  BoundingVolumeHierarchyAabb bounds;
  BoundingVolumeHierarchyAabb centroid_bounds;
  for (std::uint32_t idx = start_idx; idx < end_idx; ++idx) {
    const std::uint32_t kPrimIdx = prim_indices[idx];
    bounds.Grow(temp_aabbs[kPrimIdx]);
    const double kCx = 0.5 * (temp_aabbs[kPrimIdx].min_x + temp_aabbs[kPrimIdx].max_x);
    const double kCy = 0.5 * (temp_aabbs[kPrimIdx].min_y + temp_aabbs[kPrimIdx].max_y);
    centroid_bounds.Grow(kCx, kCy);
  }

  const std::uint32_t kCount = end_idx - start_idx;
  constexpr std::uint32_t kLeafThreshold = 4;

  if (kCount <= kLeafThreshold) {
    return MakeLeafNode(nodes, node_idx, bounds, final_primitives, prim_indices, temp_primitives, start_idx, kCount);
  }

  const double kExtX = centroid_bounds.max_x - centroid_bounds.min_x;
  const double kExtY = centroid_bounds.max_y - centroid_bounds.min_y;
  const int kAxis = (kExtX > kExtY) ? 0 : 1;

  double min_coord = (kAxis == 0) ? centroid_bounds.min_x : centroid_bounds.min_y;
  const double kMaxCoord = (kAxis == 0) ? centroid_bounds.max_x : centroid_bounds.max_y;

  if (kMaxCoord - min_coord < 1e-9) {
    return MakeLeafNode(nodes, node_idx, bounds, final_primitives, prim_indices, temp_primitives, start_idx, kCount);
  }

  constexpr int kNumBins = 16;
  struct Bin {
    std::uint32_t count{0};
    BoundingVolumeHierarchyAabb bounds;
  };
  std::array<Bin, kNumBins> bins{};

  double scale = kNumBins / (kMaxCoord - min_coord);
  for (std::uint32_t idx = start_idx; idx < end_idx; ++idx) {
    const std::uint32_t kPrimIdx = prim_indices[idx];
    const double kCentroid = (kAxis == 0) ? (0.5 * (temp_aabbs[kPrimIdx].min_x + temp_aabbs[kPrimIdx].max_x))
                                          : (0.5 * (temp_aabbs[kPrimIdx].min_y + temp_aabbs[kPrimIdx].max_y));
    int bin_idx = static_cast<int>((kCentroid - min_coord) * scale);
    bin_idx = std::clamp(bin_idx, 0, kNumBins - 1);
    bins[bin_idx].count++;
    bins[bin_idx].bounds.Grow(temp_aabbs[kPrimIdx]);
  }

  double min_split_cost = std::numeric_limits<double>::max();
  int best_split_bin = -1;

  std::array<BoundingVolumeHierarchyAabb, kNumBins - 1> left_bounds{};
  std::array<std::uint32_t, kNumBins - 1> left_counts{};
  BoundingVolumeHierarchyAabb left_accum;
  std::uint32_t left_cnt = 0;
  for (int idx = 0; idx < kNumBins - 1; ++idx) {
    left_accum.Grow(bins[idx].bounds);
    left_cnt += bins[idx].count;
    left_bounds[idx] = left_accum;
    left_counts[idx] = left_cnt;
  }

  std::array<BoundingVolumeHierarchyAabb, kNumBins - 1> right_bounds{};
  std::array<std::uint32_t, kNumBins - 1> right_counts{};
  BoundingVolumeHierarchyAabb right_accum;
  std::uint32_t right_cnt = 0;
  for (int idx = kNumBins - 1; idx > 0; --idx) {
    right_accum.Grow(bins[idx].bounds);
    right_cnt += bins[idx].count;
    right_bounds[idx - 1] = right_accum;
    right_counts[idx - 1] = right_cnt;
  }

  const double kParentArea = bounds.Area();
  constexpr double kCTrav = 1.0;
  constexpr double kCIsect = 1.0;

  for (int idx = 0; idx < kNumBins - 1; ++idx) {
    if (left_counts[idx] == 0 || right_counts[idx] == 0) {
      continue;
    }
    const double kCost =
        kCTrav +
        (kCIsect * (left_bounds[idx].Area() * left_counts[idx] + right_bounds[idx].Area() * right_counts[idx]) /
         kParentArea);
    if (kCost < min_split_cost) {
      min_split_cost = kCost;
      best_split_bin = idx;
    }
  }

  const double kNoSplitCost = kCount * kCIsect;

  if (min_split_cost >= kNoSplitCost) {
    return MakeLeafNode(nodes, node_idx, bounds, final_primitives, prim_indices, temp_primitives, start_idx, kCount);
  }

  auto split_it = std::stable_partition(
      prim_indices.begin() + start_idx, prim_indices.begin() + end_idx, [&](std::uint32_t prim_idx) -> bool {
        const double kCentroid = (kAxis == 0) ? (0.5 * (temp_aabbs[prim_idx].min_x + temp_aabbs[prim_idx].max_x))
                                              : (0.5 * (temp_aabbs[prim_idx].min_y + temp_aabbs[prim_idx].max_y));
        int bin_idx = static_cast<int>((kCentroid - min_coord) * scale);
        bin_idx = std::clamp(bin_idx, 0, kNumBins - 1);
        return bin_idx <= best_split_bin;
      });

  auto mid_idx = static_cast<std::uint32_t>(std::distance(prim_indices.begin(), split_it));

  if (mid_idx == start_idx || mid_idx == end_idx) {
    mid_idx = start_idx + (kCount / 2);
  }

  const std::uint32_t kLeftChild = BuildBoundingVolumeHierarchyRecursive(
      nodes, final_primitives, prim_indices, temp_primitives, temp_aabbs, start_idx, mid_idx);
  const std::uint32_t kRightChild = BuildBoundingVolumeHierarchyRecursive(
      nodes, final_primitives, prim_indices, temp_primitives, temp_aabbs, mid_idx, end_idx);

  nodes[node_idx].min_x = bounds.min_x;
  nodes[node_idx].min_y = bounds.min_y;
  nodes[node_idx].max_x = bounds.max_x;
  nodes[node_idx].max_y = bounds.max_y;
  nodes[node_idx].left = kLeftChild;
  nodes[node_idx].right = kRightChild;

  return node_idx;
}

}  // namespace

auto BoundingVolumeHierarchy::Build(std::vector<std::uint32_t>& prim_indices,
                                    std::span<const PrimitiveInfo> temp_primitives, std::span<const Aabb> temp_aabbs)
    -> BoundingVolumeHierarchy {
  BoundingVolumeHierarchy bounding_volume_hierarchy;
  if (temp_primitives.empty()) {
    return bounding_volume_hierarchy;
  }
  BuildBoundingVolumeHierarchyRecursive(bounding_volume_hierarchy.nodes_, bounding_volume_hierarchy.primitives_,
                                        prim_indices, temp_primitives, temp_aabbs, 0,
                                        static_cast<std::uint32_t>(temp_primitives.size()));
  return bounding_volume_hierarchy;
}

auto BoundingVolumeHierarchy::DistancePointToAabb(double px, double py, double min_x, double min_y, double max_x,
                                                  double max_y) noexcept -> double {
  const double kDx = std::max({0.0, min_x - px, px - max_x});
  const double kDy = std::max({0.0, min_y - py, py - max_y});
  return std::sqrt((kDx * kDx) + (kDy * kDy));
}

}  // namespace strada::cpm
