#pragma once

#include <cstddef>
#include <optional>
#include <strada/ast/abstract_syntax_tree.hpp>
#include <strada/cpm/aligned_allocator.hpp>
#include <strada/cpm/coordinate.hpp>
#include <strada/cpm/ids.hpp>
#include <strada/cpm/query_context.hpp>
#include <string_view>
#include <vector>

namespace strada::cpm {

constexpr std::size_t kAlignmentBytes = 64;

template <typename T>
using AlignedVector = std::vector<T, AlignedAllocator<T, kAlignmentBytes>>;

enum class GeometryType : uint8_t { kLine, kArc, kSpiral, kPoly3, kParamPoly3 };

struct ReferenceLineSoA {
  AlignedVector<double> s_offset;
  AlignedVector<double> length;
  AlignedVector<double> x;
  AlignedVector<double> y;
  AlignedVector<double> hdg;
  std::vector<GeometryType> type;
  std::vector<uint32_t> type_index;
  AlignedVector<double> spiral_curv_start;
  AlignedVector<double> spiral_curv_end;
  AlignedVector<double> pp3_a_u;
  AlignedVector<double> pp3_b_u;
  AlignedVector<double> pp3_c_u;
  AlignedVector<double> pp3_d_u;
  AlignedVector<double> pp3_a_v;
  AlignedVector<double> pp3_b_v;
  AlignedVector<double> pp3_c_v;
  AlignedVector<double> pp3_d_v;
  std::vector<uint8_t> pp3_p_range;
};

// Flat SoA structures for Cross Section Surface per ADR 0005
struct PolynomialsSoA {
  AlignedVector<double> s_start;
  AlignedVector<double> a;
  AlignedVector<double> b;
  AlignedVector<double> c;
  AlignedVector<double> d;
};

struct StripsSoA {
  std::vector<int32_t> strip_id;
  std::vector<uint8_t> is_relative;
  std::vector<uint32_t> width_first_idx;
  std::vector<uint32_t> width_count;
  std::vector<uint32_t> c0_first_idx;
  std::vector<uint32_t> c0_count;
  std::vector<uint32_t> c1_first_idx;
  std::vector<uint32_t> c1_count;
  std::vector<uint32_t> c2_first_idx;
  std::vector<uint32_t> c2_count;
  std::vector<uint32_t> c3_first_idx;
  std::vector<uint32_t> c3_count;
};

struct RoadCrossSectionSurfaceSoA {
  std::vector<uint32_t> first_strip_idx;
  std::vector<uint32_t> strip_count;
  std::vector<uint32_t> t_offset_first_idx;
  std::vector<uint32_t> t_offset_count;
};

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

class CompiledPhysicsModel {
 public:
  CompiledPhysicsModel() = default;
  ~CompiledPhysicsModel() = default;

  // Move-constructible only, per ADR 0004
  CompiledPhysicsModel(const CompiledPhysicsModel&) = delete;
  auto operator=(const CompiledPhysicsModel&) -> CompiledPhysicsModel& = delete;
  CompiledPhysicsModel(CompiledPhysicsModel&&) noexcept = default;
  auto operator=(CompiledPhysicsModel&&) noexcept -> CompiledPhysicsModel& = default;

  // Hot-path queries: noexcept, take QueryContext&.
  auto RoadToInertial(RoadPose pose, QueryContext& ctx) const noexcept -> InertialPose;
  auto LaneToInertial(LanePose pose, QueryContext& ctx) const noexcept -> InertialPose;
  auto InertialToRoad(InertialPose pose, QueryContext& ctx) const noexcept -> std::optional<RoadPose>;
  auto InertialToLane(InertialPose pose, QueryContext& ctx) const noexcept -> std::optional<LanePose>;
  auto RoadToLane(RoadPose pose, QueryContext& ctx) const noexcept -> std::optional<LanePose>;
  auto LaneToRoad(LanePose pose, QueryContext& ctx) const noexcept -> RoadPose;

  // Inspection: noexcept, stateless.
  [[nodiscard]] auto RoadCount() const noexcept -> std::size_t;

  [[nodiscard]] auto RoadIdFromString(std::string_view original_id) const noexcept -> std::optional<RoadId>;

  [[nodiscard]] auto OriginalRoadId(RoadId road_id) const noexcept -> std::string_view;

  [[nodiscard]] auto RoadLength(RoadId road_id) const noexcept -> double;

  [[nodiscard]] auto LaneCount() const noexcept -> std::size_t;

  [[nodiscard]] auto LaneRoad(LaneId lane_id) const noexcept -> RoadId;

  [[nodiscard]] auto OriginalLaneId(LaneId lane_id) const noexcept -> int;

  [[nodiscard]] auto LaneWidth(LaneId lane_id, double s_coord) const noexcept -> double;

  // BVH inspection
  [[nodiscard]] auto GetBvhNodes() const noexcept -> const std::vector<BvhNode>& { return bvh_nodes_; }
  [[nodiscard]] auto GetBvhPrimitives() const noexcept -> const std::vector<BvhPrimitiveInfo>& {
    return bvh_primitives_;
  }
  void ClearBvhNodes() noexcept { bvh_nodes_.clear(); }

 private:
  friend auto BuildCompiledPhysicsModel(const ast::AbstractSyntaxTree& map) -> CompiledPhysicsModel;

  // Road ID mapping tables
  std::vector<std::string> road_string_ids_;
  std::vector<double> road_lengths_;

  // Reference line SoA
  ReferenceLineSoA ref_line_;
  AlignedVector<double> arc_curvature_;
  std::vector<uint32_t> road_ref_line_first_idx_;
  std::vector<uint32_t> road_ref_line_count_;

  // Elevation and Superelevation indexing
  std::vector<uint32_t> road_elevation_first_idx_;
  std::vector<uint32_t> road_elevation_count_;
  std::vector<uint32_t> road_superelevation_first_idx_;
  std::vector<uint32_t> road_superelevation_count_;

  // Cross section surface flat SoA structures
  PolynomialsSoA polynomials_;
  StripsSoA strips_;
  RoadCrossSectionSurfaceSoA road_css_;

  // Lane sections flat SoA structures (CSR-style)
  std::vector<double> section_s_;
  std::vector<uint32_t> section_first_lane_idx_;
  std::vector<uint32_t> section_lane_count_;
  std::vector<uint32_t> road_section_first_idx_;
  std::vector<uint32_t> road_section_count_;

  // Lanes flat SoA structures
  std::vector<int> lane_original_id_;
  std::vector<RoadId> lane_road_id_;
  std::vector<uint32_t> lane_section_idx_;
  std::vector<uint32_t> lane_first_width_idx_;
  std::vector<uint32_t> lane_width_count_;
  std::vector<uint32_t> lane_first_height_idx_;
  std::vector<uint32_t> lane_height_count_;

  // Lane widths flat SoA structures
  std::vector<double> lane_width_s_start_;
  std::vector<double> lane_width_a_;
  std::vector<double> lane_width_b_;
  std::vector<double> lane_width_c_;
  std::vector<double> lane_width_d_;

  // Lane heights flat SoA structures
  std::vector<double> lane_height_s_start_;
  std::vector<double> lane_height_inner_;
  std::vector<double> lane_height_outer_;

  // Lane offsets flat SoA structures (road level)
  std::vector<double> lane_offset_s_start_;
  std::vector<double> lane_offset_a_;
  std::vector<double> lane_offset_b_;
  std::vector<double> lane_offset_c_;
  std::vector<double> lane_offset_d_;
  std::vector<uint32_t> road_lane_offset_first_idx_;
  std::vector<uint32_t> road_lane_offset_count_;

  // Global spatial index (Flat BVH)
  std::vector<BvhNode> bvh_nodes_;
  std::vector<BvhPrimitiveInfo> bvh_primitives_;

  void GetRoadWidthLimits(uint32_t road_idx, double s_coord, double& t_left, double& t_right) const noexcept;
};

auto BuildCompiledPhysicsModel(const ast::AbstractSyntaxTree& map) -> CompiledPhysicsModel;

}  // namespace strada::cpm
