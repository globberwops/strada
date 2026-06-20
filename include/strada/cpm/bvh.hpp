#pragma once

#include <cstdint>
#include <vector>
#include <array>
#include <optional>
#include <limits>
#include <strada/cpm/coordinate.hpp>
#include <strada/cpm/reference_line.hpp>

namespace strada::cpm {

struct BvhNode {
  double min_x{};
  double min_y{};
  double max_x{};
  double max_y{};
  uint32_t left{};
  uint32_t right{};
};
static_assert(sizeof(BvhNode) == 40, "BvhNode must be exactly 40 bytes");

struct BvhPrimitiveInfo {
  uint32_t road_idx{};
  uint32_t segment_idx{};
};

class Bvh {
 public:
  Bvh() = default;

  static auto Build(std::vector<uint32_t>& prim_indices,
                    const std::vector<BvhPrimitiveInfo>& temp_primitives,
                    const std::vector<Aabb>& temp_aabbs) -> Bvh;

  template <typename F>
  void Query(double px, double py, F&& callback) const noexcept {
    if (nodes_.empty()) {
      return;
    }

    std::array<uint32_t, 64> stack{};
    int stack_ptr = 0;
    stack[stack_ptr++] = 0;

    double min_distance = std::numeric_limits<double>::max();

    while (stack_ptr > 0) {
      auto curr_idx = stack[--stack_ptr];
      const auto& node = nodes_[curr_idx];

      double dist_to_box = DistancePointToAabb(px, py, node.min_x, node.min_y, node.max_x, node.max_y);
      if (dist_to_box > min_distance) {
        continue;
      }

      bool is_leaf = (node.right & 0x80000000) != 0;
      if (is_leaf) {
        auto prim_start = node.left;
        auto prim_count = node.right & 0x7FFFFFFF;

        for (uint32_t i = 0; i < prim_count; ++i) {
          const auto& prim = primitives_[prim_start + i];
          if (auto new_dist = callback(prim, min_distance)) {
            min_distance = *new_dist;
          }
        }
      } else {
        auto left_child = node.left;
        auto right_child = node.right & 0x7FFFFFFF;

        double dist_left = DistancePointToAabb(px, py, nodes_[left_child].min_x, nodes_[left_child].min_y,
                                               nodes_[left_child].max_x, nodes_[left_child].max_y);
        double dist_right =
            DistancePointToAabb(px, py, nodes_[right_child].min_x, nodes_[right_child].min_y,
                                nodes_[right_child].max_x, nodes_[right_child].max_y);

        if (dist_left < dist_right) {
          stack[stack_ptr++] = right_child;
          stack[stack_ptr++] = left_child;
        } else {
          stack[stack_ptr++] = left_child;
          stack[stack_ptr++] = right_child;
        }
      }
    }
  }

  [[nodiscard]] auto Nodes() const noexcept -> const std::vector<BvhNode>& { return nodes_; }
  [[nodiscard]] auto Primitives() const noexcept -> const std::vector<BvhPrimitiveInfo>& { return primitives_; }
  void Clear() noexcept { nodes_.clear(); }

 private:
  std::vector<BvhNode> nodes_;
  std::vector<BvhPrimitiveInfo> primitives_;

  static auto DistancePointToAabb(double px, double py, double min_x, double min_y, double max_x, double max_y) noexcept -> double;
};

} // namespace strada::cpm
