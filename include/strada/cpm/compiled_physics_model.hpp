#pragma once

#include <cstddef>
#include <optional>
#include <strada/ast/abstract_syntax_tree.hpp>
#include <strada/cpm/aligned_allocator.hpp>
#include <strada/cpm/bounding_volume_hierarchy.hpp>
#include <strada/cpm/coordinate.hpp>
#include <strada/cpm/ids.hpp>
#include <strada/cpm/query_context.hpp>
#include <strada/cpm/reference_line.hpp>
#include <string_view>
#include <vector>

namespace strada::cpm {

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

struct ShapesSoA {
  AlignedVector<double> s;
  AlignedVector<double> t;
  AlignedVector<double> a;
  AlignedVector<double> b;
  AlignedVector<double> c;
  AlignedVector<double> d;
  std::vector<uint32_t> road_shape_first_idx;
  std::vector<uint32_t> road_shape_count;
};

struct ElevationSoA {
  std::vector<uint32_t> road_elevation_first_idx;
  std::vector<uint32_t> road_elevation_count;
  std::vector<uint32_t> road_superelevation_first_idx;
  std::vector<uint32_t> road_superelevation_count;
};

struct LaneSectionsSoA {
  std::vector<double> section_s;
  std::vector<uint32_t> section_first_lane_idx;
  std::vector<uint32_t> section_lane_count;
  std::vector<uint32_t> road_section_first_idx;
  std::vector<uint32_t> road_section_count;
};

struct LanesSoA {
  std::vector<int> lane_original_id;
  std::vector<RoadId> lane_road_id;
  std::vector<uint32_t> lane_section_idx;
  std::vector<uint32_t> lane_first_width_idx;
  std::vector<uint32_t> lane_width_count;
  std::vector<uint32_t> lane_first_height_idx;
  std::vector<uint32_t> lane_height_count;
};

struct LaneWidthsSoA {
  std::vector<double> lane_width_s_start;
  std::vector<double> lane_width_a;
  std::vector<double> lane_width_b;
  std::vector<double> lane_width_c;
  std::vector<double> lane_width_d;
};

struct LaneHeightsSoA {
  std::vector<double> lane_height_s_start;
  std::vector<double> lane_height_inner;
  std::vector<double> lane_height_outer;
};

struct LaneOffsetsSoA {
  std::vector<double> lane_offset_s_start;
  std::vector<double> lane_offset_a;
  std::vector<double> lane_offset_b;
  std::vector<double> lane_offset_c;
  std::vector<double> lane_offset_d;
  std::vector<uint32_t> road_lane_offset_first_idx;
  std::vector<uint32_t> road_lane_offset_count;
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

  // Bounding volume hierarchy inspection
  [[nodiscard]] auto GetBoundingVolumeHierarchyNodes() const noexcept
      -> const std::vector<BoundingVolumeHierarchy::Node>& {
    return bounding_volume_hierarchy_.Nodes();
  }
  [[nodiscard]] auto GetBoundingVolumeHierarchyPrimitives() const noexcept
      -> const std::vector<BoundingVolumeHierarchy::PrimitiveInfo>& {
    return bounding_volume_hierarchy_.Primitives();
  }
  auto ClearBoundingVolumeHierarchyNodes() noexcept -> void { bounding_volume_hierarchy_.Clear(); }

 private:
  friend auto BuildCompiledPhysicsModel(const ast::AbstractSyntaxTree& map) -> CompiledPhysicsModel;

  // Road ID mapping tables
  std::vector<std::string> road_string_ids_;
  std::vector<double> road_lengths_;

  // Reference line
  ReferenceLine ref_line_;

  // Elevation and Superelevation indexing
  ElevationSoA elevation_;

  // Cross section surface flat SoA structures
  PolynomialsSoA polynomials_;
  StripsSoA strips_;
  RoadCrossSectionSurfaceSoA road_css_;

  // Lane sections flat SoA structures (CSR-style)
  LaneSectionsSoA lane_sections_;

  // Lanes flat SoA structures
  LanesSoA lanes_;

  // Lane widths flat SoA structures
  LaneWidthsSoA lane_widths_;

  // Lane heights flat SoA structures
  LaneHeightsSoA lane_heights_;

  // Lane offsets flat SoA structures (road level)
  LaneOffsetsSoA lane_offsets_;

  // Shape profile flat SoA structures
  ShapesSoA shapes_;

  // Global spatial index (Flat Bounding Volume Hierarchy)
  BoundingVolumeHierarchy bounding_volume_hierarchy_;

  void GetRoadWidthLimits(uint32_t road_idx, double s_coord, double& t_left, double& t_right) const noexcept;
};

auto BuildCompiledPhysicsModel(const ast::AbstractSyntaxTree& map) -> CompiledPhysicsModel;

}  // namespace strada::cpm
