#pragma once

#include <array>
#include <cstdint>
#include <limits>
#include <span>
#include <strada/cpm/coordinate.hpp>
#include <strada/cpm/reference_line.hpp>
#include <vector>

namespace strada::cpm {

/// Bounding Volume Hierarchy (BVH) spatial index.
///
/// This class builds a flat, contiguous 2D bounding volume hierarchy over plan-view
/// axis-aligned bounding boxes (AABBs) for fast road/segment lookup.
class BoundingVolumeHierarchy {
 public:
  static constexpr std::uint32_t kLeafBitMask = 0x80000000;
  static constexpr std::uint32_t kIndexBitMask = 0x7FFFFFFF;
  static constexpr std::size_t kMaxStackDepth = 64;
  static constexpr std::size_t kExpectedNodeSize = 40;

  /// Represents a flat node in the bounding volume hierarchy.
  struct Node {
    double min_x{};         ///< Minimum x coordinate of the node's bounding box.
    double min_y{};         ///< Minimum y coordinate of the node's bounding box.
    double max_x{};         ///< Maximum x coordinate of the node's bounding box.
    double max_y{};         ///< Maximum y coordinate of the node's bounding box.
    std::uint32_t left{};   ///< For leaf nodes: primitive start index. For internal nodes: left child index.
    std::uint32_t right{};  ///< For leaf nodes: primitive count with MSB set. For internal nodes: right child index.
  };

  /// Represents association mapping of a leaf primitive to the road and segment.
  struct PrimitiveInfo {
    std::uint32_t road_idx{};     ///< Index of the road in the compiled physics model.
    std::uint32_t segment_idx{};  ///< Index of the road segment in the reference line.
  };

  /// Default-constructs an empty BoundingVolumeHierarchy.
  BoundingVolumeHierarchy() = default;

  /// Destructor.
  ~BoundingVolumeHierarchy() = default;

  // Move-only semantics
  BoundingVolumeHierarchy(const BoundingVolumeHierarchy&) = delete;
  auto operator=(const BoundingVolumeHierarchy&) -> BoundingVolumeHierarchy& = delete;
  BoundingVolumeHierarchy(BoundingVolumeHierarchy&&) noexcept = default;
  auto operator=(BoundingVolumeHierarchy&&) noexcept -> BoundingVolumeHierarchy& = default;

  /// Constructs a BoundingVolumeHierarchy from a set of primitives and their AABBs.
  ///
  /// \param prim_indices A list of primitive indices that will be partition-sorted during build.
  /// \param temp_primitives A view over the source primitives info.
  /// \param temp_aabbs A view over the corresponding source plan-view AABBs.
  explicit BoundingVolumeHierarchy(std::vector<std::uint32_t>& prim_indices,
                                   std::span<const PrimitiveInfo> temp_primitives, std::span<const Aabb> temp_aabbs);

  /// Queries the hierarchy for primitives that contain or are close to a given point.
  ///
  /// Traverses the tree using a stack and invokes the provided callback on overlapping primitives.
  ///
  /// \param px The target point's x coordinate.
  /// \param py The target point's y coordinate.
  /// \param callback A callable invoked for each overlapping leaf primitive. It should have the
  ///        signature `(const PrimitiveInfo&, double current_min_dist) -> std::optional<double>`.
  template <typename F>
  void Query(double px, double py, F&& callback) const noexcept {
    if (nodes_.empty()) {
      return;
    }

    std::array<std::uint32_t, kMaxStackDepth> stack{};
    std::size_t stack_ptr{0};
    stack[stack_ptr++] = 0;  // NOLINT(cppcoreguidelines-pro-bounds-constant-array-index)

    double min_distance{std::numeric_limits<double>::max()};

    while (stack_ptr > 0) {
      auto curr_idx = stack[--stack_ptr];  // NOLINT(cppcoreguidelines-pro-bounds-constant-array-index)
      const auto& node = nodes_[curr_idx];

      const double dist_to_box = DistancePointToAabb(px, py, node.min_x, node.min_y, node.max_x, node.max_y);
      if (dist_to_box > min_distance) {
        continue;
      }

      const bool is_leaf = (node.right & kLeafBitMask) != 0;
      if (is_leaf) {
        const auto prim_start = node.left;
        const auto prim_count = node.right & kIndexBitMask;

        for (std::uint32_t i = 0; i < prim_count; ++i) {
          const auto& prim = primitives_[prim_start + i];
          if (auto new_dist = callback(prim, min_distance)) {
            min_distance = *new_dist;
          }
        }
      } else {
        const auto left_child = node.left;
        const auto right_child = node.right & kIndexBitMask;

        const double dist_left = DistancePointToAabb(px, py, nodes_[left_child].min_x, nodes_[left_child].min_y,
                                                     nodes_[left_child].max_x, nodes_[left_child].max_y);
        const double dist_right = DistancePointToAabb(px, py, nodes_[right_child].min_x, nodes_[right_child].min_y,
                                                      nodes_[right_child].max_x, nodes_[right_child].max_y);

        if (dist_left < dist_right) {
          stack[stack_ptr++] = right_child;  // NOLINT(cppcoreguidelines-pro-bounds-constant-array-index)
          stack[stack_ptr++] = left_child;   // NOLINT(cppcoreguidelines-pro-bounds-constant-array-index)
        } else {
          stack[stack_ptr++] = left_child;   // NOLINT(cppcoreguidelines-pro-bounds-constant-array-index)
          stack[stack_ptr++] = right_child;  // NOLINT(cppcoreguidelines-pro-bounds-constant-array-index)
        }
      }
    }
  }

  /// Returns a reference to the flat vector of nodes.
  ///
  /// \return The contiguous array of hierarchy nodes.
  [[nodiscard]] auto Nodes() const noexcept -> const std::vector<Node>& { return nodes_; }

  /// Returns a reference to the flat vector of leaf primitives.
  ///
  /// \return The contiguous array of primitives in leaf order.
  [[nodiscard]] auto Primitives() const noexcept -> const std::vector<PrimitiveInfo>& { return primitives_; }

  /// Clears the hierarchy, releasing or clearing the nodes.
  void Clear() noexcept { nodes_.clear(); }

 private:
  std::vector<Node> nodes_;
  std::vector<PrimitiveInfo> primitives_;

  static auto DistancePointToAabb(double px, double py, double min_x, double min_y, double max_x, double max_y) noexcept
      -> double;
};

static_assert(sizeof(BoundingVolumeHierarchy::Node) == BoundingVolumeHierarchy::kExpectedNodeSize,
              "BoundingVolumeHierarchy::Node must be exactly 40 bytes");

}  // namespace strada::cpm
