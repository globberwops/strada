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

/// Represents a flat Structure of Arrays (SoA) for polynomial coefficients.
struct PolynomialsSoA {
  AlignedVector<double> s_start;  ///< Start s-coordinate of the polynomial's valid range.
  AlignedVector<double> a;        ///< Constant coefficient (a).
  AlignedVector<double> b;        ///< Linear coefficient (b).
  AlignedVector<double> c;        ///< Quadratic coefficient (c).
  AlignedVector<double> d;        ///< Cubic coefficient (d).
};

/// Represents a flat Structure of Arrays (SoA) for cross-section strips.
struct StripsSoA {
  std::vector<int32_t> strip_id;          ///< Unique ID of each strip.
  std::vector<uint8_t> is_relative;       ///< Flag indicating if width is relative.
  std::vector<uint32_t> width_first_idx;  ///< Start index in width polynomials.
  std::vector<uint32_t> width_count;      ///< Count of width polynomials.
  std::vector<uint32_t> c0_first_idx;     ///< Start index in c0 shape polynomials.
  std::vector<uint32_t> c0_count;         ///< Count of c0 shape polynomials.
  std::vector<uint32_t> c1_first_idx;     ///< Start index in c1 shape polynomials.
  std::vector<uint32_t> c1_count;         ///< Count of c1 shape polynomials.
  std::vector<uint32_t> c2_first_idx;     ///< Start index in c2 shape polynomials.
  std::vector<uint32_t> c2_count;         ///< Count of c2 shape polynomials.
  std::vector<uint32_t> c3_first_idx;     ///< Start index in c3 shape polynomials.
  std::vector<uint32_t> c3_count;         ///< Count of c3 shape polynomials.
};

/// Represents a flat Structure of Arrays (SoA) for road-level cross-section surfaces.
struct RoadCrossSectionSurfaceSoA {
  std::vector<uint32_t> first_strip_idx;     ///< Start index of the road's strips.
  std::vector<uint32_t> strip_count;         ///< Count of the road's strips.
  std::vector<uint32_t> t_offset_first_idx;  ///< Start index of the lateral offset polynomials.
  std::vector<uint32_t> t_offset_count;      ///< Count of the lateral offset polynomials.
};

/// Represents a flat Structure of Arrays (SoA) for shape profile parameters.
struct ShapesSoA {
  AlignedVector<double> s;                     ///< Start s-coordinate of the shape.
  AlignedVector<double> t;                     ///< Start t-coordinate of the shape.
  AlignedVector<double> a;                     ///< Constant coefficient (a).
  AlignedVector<double> b;                     ///< Linear coefficient (b).
  AlignedVector<double> c;                     ///< Quadratic coefficient (c).
  AlignedVector<double> d;                     ///< Cubic coefficient (d).
  std::vector<uint32_t> road_shape_first_idx;  ///< Start index of road shapes.
  std::vector<uint32_t> road_shape_count;      ///< Count of road shapes.
};

/// Represents a flat Structure of Arrays (SoA) for road elevation and superelevation.
struct ElevationSoA {
  std::vector<uint32_t> road_elevation_first_idx;       ///< Start index in elevation polynomials.
  std::vector<uint32_t> road_elevation_count;           ///< Count of elevation polynomials.
  std::vector<uint32_t> road_superelevation_first_idx;  ///< Start index in superelevation polynomials.
  std::vector<uint32_t> road_superelevation_count;      ///< Count of superelevation polynomials.
};

/// Represents a flat Structure of Arrays (SoA) for lane sections (CSR format).
struct LaneSectionsSoA {
  std::vector<double> section_s;                 ///< Start s-coordinate of each section.
  std::vector<uint32_t> section_first_lane_idx;  ///< Start index of section's lanes.
  std::vector<uint32_t> section_lane_count;      ///< Count of section's lanes.
  std::vector<uint32_t> road_section_first_idx;  ///< Start index of road sections.
  std::vector<uint32_t> road_section_count;      ///< Count of road sections.
};

/// Represents a flat Structure of Arrays (SoA) for lanes.
struct LanesSoA {
  std::vector<int> lane_original_id;            ///< Original integer ID from parser.
  std::vector<RoadId> lane_road_id;             ///< Reference to the parent road.
  std::vector<uint32_t> lane_section_idx;       ///< Reference to the lane section index.
  std::vector<uint32_t> lane_first_width_idx;   ///< Start index in lane width polynomials.
  std::vector<uint32_t> lane_width_count;       ///< Count of lane width polynomials.
  std::vector<uint32_t> lane_first_height_idx;  ///< Start index in lane height polynomials.
  std::vector<uint32_t> lane_height_count;      ///< Count of lane height polynomials.
};

/// Represents a flat Structure of Arrays (SoA) for lane width profiles.
struct LaneWidthsSoA {
  std::vector<double> lane_width_s_start;  ///< Start s-coordinate of width profile.
  std::vector<double> lane_width_a;        ///< Constant coefficient (a).
  std::vector<double> lane_width_b;        ///< Linear coefficient (b).
  std::vector<double> lane_width_c;        ///< Quadratic coefficient (c).
  std::vector<double> lane_width_d;        ///< Cubic coefficient (d).
};

/// Represents a flat Structure of Arrays (SoA) for lane height profiles.
struct LaneHeightsSoA {
  std::vector<double> lane_height_s_start;  ///< Start s-coordinate of height profile.
  std::vector<double> lane_height_inner;    ///< Inner height offset.
  std::vector<double> lane_height_outer;    ///< Outer height offset.
};

/// Represents a flat Structure of Arrays (SoA) for lane offsets (road level).
struct LaneOffsetsSoA {
  std::vector<double> lane_offset_s_start;           ///< Start s-coordinate of offset profile.
  std::vector<double> lane_offset_a;                 ///< Constant coefficient (a).
  std::vector<double> lane_offset_b;                 ///< Linear coefficient (b).
  std::vector<double> lane_offset_c;                 ///< Quadratic coefficient (c).
  std::vector<double> lane_offset_d;                 ///< Cubic coefficient (d).
  std::vector<uint32_t> road_lane_offset_first_idx;  ///< Start index of road offsets.
  std::vector<uint32_t> road_lane_offset_count;      ///< Count of road offsets.
};

/// Optimised, flat Structure-of-Arrays (SoA) representation of the road network physics model.
///
/// This class holds compiled representation of plan-view geometries, lane sections,
/// lane profiles, and lateral cross-sections. It is designed to expose hot-path
/// coordinate conversions and queries in a performant, thread-safe manner.
class CompiledPhysicsModel {
 public:
  /// Default-constructs a CompiledPhysicsModel.
  CompiledPhysicsModel() = default;

  /// Destructs a CompiledPhysicsModel.
  ~CompiledPhysicsModel() = default;

  // Move-constructible only, per ADR 0004
  CompiledPhysicsModel(const CompiledPhysicsModel&) = delete;
  auto operator=(const CompiledPhysicsModel&) -> CompiledPhysicsModel& = delete;
  CompiledPhysicsModel(CompiledPhysicsModel&&) noexcept = default;
  auto operator=(CompiledPhysicsModel&&) noexcept -> CompiledPhysicsModel& = default;

  /// Converts a pose from road-local coordinates (s, t, h) to global inertial coordinates (x, y, z).
  ///
  /// \param pose The source pose in road-local coordinates.
  /// \param ctx The query context used to exploit temporal coherence.
  /// \return The resulting global inertial pose.
  auto RoadToInertial(RoadPose pose, QueryContext& ctx) const noexcept -> InertialPose;

  /// Converts a pose from lane-local coordinates (s, t, h) to global inertial coordinates (x, y, z).
  ///
  /// \param pose The source pose in lane-local coordinates.
  /// \param ctx The query context used to exploit temporal coherence.
  /// \return The resulting global inertial pose.
  auto LaneToInertial(LanePose pose, QueryContext& ctx) const noexcept -> InertialPose;

  /// Converts a pose from global inertial coordinates (x, y, z) to road-local coordinates (s, t, h).
  ///
  /// \param pose The source pose in global inertial coordinates.
  /// \param ctx The query context used to exploit temporal coherence.
  /// \return The matched road pose, or std::nullopt if the point is outside the spatial index.
  auto InertialToRoad(InertialPose pose, QueryContext& ctx) const noexcept -> std::optional<RoadPose>;

  /// Converts a pose from global inertial coordinates (x, y, z) to lane-local coordinates (s, t, h).
  ///
  /// \param pose The source pose in global inertial coordinates.
  /// \param ctx The query context used to exploit temporal coherence.
  /// \return The matched lane pose, or std::nullopt if the point is outside the spatial index.
  auto InertialToLane(InertialPose pose, QueryContext& ctx) const noexcept -> std::optional<LanePose>;

  /// Translates a road-local pose to the corresponding lane-local pose.
  ///
  /// \param pose The source pose in road-local coordinates.
  /// \param ctx The query context used to exploit temporal coherence.
  /// \return The matched lane pose, or std::nullopt if the point does not lie in any lane.
  auto RoadToLane(RoadPose pose, QueryContext& ctx) const noexcept -> std::optional<LanePose>;

  /// Translates a lane-local pose to the corresponding road-local pose.
  ///
  /// \param pose The source pose in lane-local coordinates.
  /// \param ctx The query context used to exploit temporal coherence.
  /// \return The corresponding road-local pose.
  auto LaneToRoad(LanePose pose, QueryContext& ctx) const noexcept -> RoadPose;

  /// Returns the total number of roads compile-mapped in the network.
  ///
  /// \return The number of roads.
  [[nodiscard]] auto RoadCount() const noexcept -> std::size_t;

  /// Resolves the integer RoadId corresponding to the parser's road string ID.
  ///
  /// \param original_id The road string ID parsed from the XODR source.
  /// \return The unique RoadId if found, or std::nullopt if not mapped.
  [[nodiscard]] auto RoadIdFromString(std::string_view original_id) const noexcept -> std::optional<RoadId>;

  /// Retrieves the original road string ID corresponding to the numeric RoadId.
  ///
  /// \param road_id The compiled RoadId.
  /// \return The XODR road string ID.
  [[nodiscard]] auto OriginalRoadId(RoadId road_id) const noexcept -> std::string_view;

  /// Returns the reference line length of the specified road.
  ///
  /// \param road_id The compiled RoadId.
  /// \return The road's length.
  [[nodiscard]] auto RoadLength(RoadId road_id) const noexcept -> double;

  /// Returns the total number of lanes compile-mapped in the network.
  ///
  /// \return The number of lanes.
  [[nodiscard]] auto LaneCount() const noexcept -> std::size_t;

  /// Retrieves the parent RoadId of the specified compiled LaneId.
  ///
  /// \param lane_id The compiled LaneId.
  /// \return The parent road's RoadId.
  [[nodiscard]] auto LaneRoad(LaneId lane_id) const noexcept -> RoadId;

  /// Retrieves the original XODR lane integer ID corresponding to the numeric LaneId.
  ///
  /// \param lane_id The compiled LaneId.
  /// \return The original lane ID from the XODR file.
  [[nodiscard]] auto OriginalLaneId(LaneId lane_id) const noexcept -> int;

  /// Computes the width of the specified lane at a given s-coordinate along its parent road.
  ///
  /// \param lane_id The compiled LaneId.
  /// \param s_coord The s-coordinate along the reference line.
  /// \return The computed lane width.
  [[nodiscard]] auto LaneWidth(LaneId lane_id, double s_coord) const noexcept -> double;

  /// Returns a reference to the flat vector of nodes in the spatial bounding volume hierarchy.
  ///
  /// \return The contiguous array of hierarchy nodes.
  [[nodiscard]] auto GetBoundingVolumeHierarchyNodes() const noexcept
      -> const std::vector<BoundingVolumeHierarchy::Node>& {
    return bounding_volume_hierarchy_.Nodes();
  }

  /// Returns a reference to the flat vector of leaf primitives in the spatial bounding volume hierarchy.
  ///
  /// \return The contiguous array of leaf primitives.
  [[nodiscard]] auto GetBoundingVolumeHierarchyPrimitives() const noexcept
      -> const std::vector<BoundingVolumeHierarchy::PrimitiveInfo>& {
    return bounding_volume_hierarchy_.Primitives();
  }

  /// Clears the bounding volume hierarchy node and primitive tables.
  void ClearBoundingVolumeHierarchyNodes() noexcept { bounding_volume_hierarchy_.Clear(); }

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

/// Compiles the abstract syntax tree (AST) into a highly optimised CompiledPhysicsModel.
///
/// \param map The parsed AST of the road network map.
/// \return The compiled physics model ready for query execution.
auto BuildCompiledPhysicsModel(const ast::AbstractSyntaxTree& map) -> CompiledPhysicsModel;

}  // namespace strada::cpm
