// SPDX-License-Identifier: BSL-1.0

#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <strada/ast/abstract_syntax_tree.hpp>
#include <strada/cpm/coordinate.hpp>
#include <strada/cpm/ids.hpp>
#include <strada/cpm/polynomials.hpp>
#include <strada/cpm/query_context.hpp>
#include <vector>

namespace strada::cpm {

/// Represents a flat Structure of Arrays (SoA) for cross-section strips.
struct StripsSoA {
  std::vector<std::int32_t> strip_id;          ///< Unique ID of each strip.
  std::vector<std::uint8_t> is_relative;       ///< Flag indicating if width is relative.
  std::vector<std::uint32_t> width_first_idx;  ///< Start index in width polynomials.
  std::vector<std::uint32_t> width_count;      ///< Count of width polynomials.
  std::vector<std::uint32_t> c0_first_idx;     ///< Start index in c0 shape polynomials.
  std::vector<std::uint32_t> c0_count;         ///< Count of c0 shape polynomials.
  std::vector<std::uint32_t> c1_first_idx;     ///< Start index in c1 shape polynomials.
  std::vector<std::uint32_t> c1_count;         ///< Count of c1 shape polynomials.
  std::vector<std::uint32_t> c2_first_idx;     ///< Start index in c2 shape polynomials.
  std::vector<std::uint32_t> c2_count;         ///< Count of c2 shape polynomials.
  std::vector<std::uint32_t> c3_first_idx;     ///< Start index in c3 shape polynomials.
  std::vector<std::uint32_t> c3_count;         ///< Count of c3 shape polynomials.
};

/// Represents a flat Structure of Arrays (SoA) for road-level cross-section surfaces.
struct RoadCrossSectionSurfaceSoA {
  std::vector<std::uint32_t> first_strip_idx;     ///< Start index of the road's strips.
  std::vector<std::uint32_t> strip_count;         ///< Count of the road's strips.
  std::vector<std::uint32_t> t_offset_first_idx;  ///< Start index of the lateral offset polynomials.
  std::vector<std::uint32_t> t_offset_count;      ///< Count of the lateral offset polynomials.
};

/// Represents a flat Structure of Arrays (SoA) for lane sections (CSR format).
struct LaneSectionsSoA {
  std::vector<double> section_s;                      ///< Start s-coordinate of each section.
  std::vector<std::uint32_t> section_first_lane_idx;  ///< Start index of section's lanes.
  std::vector<std::uint32_t> section_lane_count;      ///< Count of section's lanes.
  std::vector<std::uint32_t> road_section_first_idx;  ///< Start index of road sections.
  std::vector<std::uint32_t> road_section_count;      ///< Count of road sections.
};

/// Represents a flat Structure of Arrays (SoA) for lanes.
struct LanesSoA {
  std::vector<int> lane_original_id;                 ///< Original integer ID from parser.
  std::vector<RoadId> lane_road_id;                  ///< Reference to the parent road.
  std::vector<std::uint32_t> lane_section_idx;       ///< Reference to the lane section index.
  std::vector<std::uint32_t> lane_first_width_idx;   ///< Start index in lane width polynomials.
  std::vector<std::uint32_t> lane_width_count;       ///< Count of lane width polynomials.
  std::vector<std::uint32_t> lane_first_height_idx;  ///< Start index in lane height polynomials.
  std::vector<std::uint32_t> lane_height_count;      ///< Count of lane height polynomials.
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
  std::vector<double> lane_offset_s_start;                ///< Start s-coordinate of offset profile.
  std::vector<double> lane_offset_a;                      ///< Constant coefficient (a).
  std::vector<double> lane_offset_b;                      ///< Linear coefficient (b).
  std::vector<double> lane_offset_c;                      ///< Quadratic coefficient (c).
  std::vector<double> lane_offset_d;                      ///< Cubic coefficient (d).
  std::vector<std::uint32_t> road_lane_offset_first_idx;  ///< Start index of road offsets.
  std::vector<std::uint32_t> road_lane_offset_count;      ///< Count of road offsets.
};

/// Compiled representation of the lane network topology and profile curves.
class LaneNetwork {
 public:
  /// Default constructor.
  LaneNetwork() = default;

  /// Destructor.
  ~LaneNetwork() = default;

  // Move-only semantics
  LaneNetwork(const LaneNetwork&) = delete;
  auto operator=(const LaneNetwork&) -> LaneNetwork& = delete;
  LaneNetwork(LaneNetwork&&) noexcept = default;
  auto operator=(LaneNetwork&&) noexcept -> LaneNetwork& = default;

  /// Builds the LaneNetwork database from the map abstract syntax tree.
  ///
  /// \param map The map AST.
  /// \return Compiled LaneNetwork.
  static auto Build(const ast::AbstractSyntaxTree& map) -> LaneNetwork;

  /// Translates a road-local pose to the corresponding lane-local pose.
  ///
  /// \param pose The source pose in road-local coordinates.
  /// \param ctx The query context used to exploit temporal coherence.
  /// \return The matched lane pose, or std::nullopt if the point does not lie in any lane.
  [[nodiscard]] auto RoadToLane(RoadPose pose, QueryContext& ctx) const noexcept -> std::optional<LanePose>;

  /// Translates a lane-local pose to the corresponding road-local pose.
  ///
  /// \param pose The source pose in lane-local coordinates.
  /// \param ctx The query context used to exploit temporal coherence.
  /// \return The corresponding road-local pose.
  [[nodiscard]] auto LaneToRoad(LanePose pose, QueryContext& ctx) const noexcept -> RoadPose;

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

  /// Evaluates cross section surface height offset at a given coordinate.
  ///
  /// \param road The compiled RoadId.
  /// \param s_coord The s-coordinate.
  /// \param t_coord The lateral t-offset.
  /// \return The cross section surface height offset.
  [[nodiscard]] auto EvaluateCrossSectionSurfaceOffset(RoadId road, double s_coord, double t_coord) const noexcept
      -> double;

  /// Computes road width limits (t_left and t_right) at a given s-coordinate.
  ///
  /// \param road The compiled RoadId.
  /// \param s_coord The s-coordinate.
  /// \param t_left Output left lateral limit.
  /// \param t_right Output right lateral limit.
  void GetRoadWidthLimits(RoadId road, double s_coord, double& t_left, double& t_right) const noexcept;

 private:
  LanesSoA lanes_;
  LaneSectionsSoA lane_sections_;
  LaneWidthsSoA lane_widths_;
  LaneHeightsSoA lane_heights_;
  LaneOffsetsSoA lane_offsets_;
  RoadCrossSectionSurfaceSoA road_css_;
  StripsSoA strips_;
  Polynomials polynomials_;
};

}  // namespace strada::cpm
