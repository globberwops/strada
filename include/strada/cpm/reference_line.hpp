#pragma once

#include <cstddef>
#include <cstdint>
#include <strada/ast/abstract_syntax_tree.hpp>
#include <strada/cpm/aligned_allocator.hpp>
#include <strada/cpm/coordinate.hpp>
#include <strada/cpm/ids.hpp>
#include <strada/cpm/query_context.hpp>
#include <utility>
#include <vector>

namespace strada::cpm {

/// Default memory boundary alignment bytes (64 bytes).
constexpr std::size_t kAlignmentBytes = 64;

/// STL vector allocator aligned to `kAlignmentBytes`.
template <typename T>
using AlignedVector = std::vector<T, AlignedAllocator<T, kAlignmentBytes>>;

/// Represents the plan-view geometry types parsed from the XODR.
enum class GeometryType : std::uint8_t {
  kLine,       ///< Straight line geometry.
  kArc,        ///< Constant curvature circular arc geometry.
  kSpiral,     ///< Clothoid (Euler spiral) geometry.
  kPoly3,      ///< Standard cubic polynomial geometry.
  kParamPoly3  ///< Parametric cubic polynomial geometry.
};

/// Represents an evaluated point on a reference line.
struct ReferenceLinePoint {
  double x{};        ///< The x coordinate (meters).
  double y{};        ///< The y coordinate (meters).
  double heading{};  ///< The heading angle (radians).
};

/// Axis-Aligned Bounding Box (AABB) in 2D.
struct Aabb {
  double min_x{};  ///< Minimum x coordinate.
  double min_y{};  ///< Minimum y coordinate.
  double max_x{};  ///< Maximum x coordinate.
  double max_y{};  ///< Maximum y coordinate.
};

/// Compiled reference line geometries of the road network map.
///
/// Holds flat SoA representation of plan-view geometries (lines, arcs, spirals, polynomials)
/// for all roads in the network, supporting evaluation, projection, and segment lookup.
class ReferenceLine {
 public:
  /// Default constructor.
  ReferenceLine() = default;

  /// Destructor.
  ~ReferenceLine() = default;

  /// Constructs ReferenceLine from the AST.
  ///
  /// \param map The parsed AST of the road network map.
  explicit ReferenceLine(const ast::AbstractSyntaxTree& map);

  // Move-only semantics
  ReferenceLine(const ReferenceLine&) = delete;
  auto operator=(const ReferenceLine&) -> ReferenceLine& = delete;
  ReferenceLine(ReferenceLine&&) noexcept = default;
  auto operator=(ReferenceLine&&) noexcept -> ReferenceLine& = default;

  /// Evaluates reference line coordinates (x, y, heading) at a road-local s-coordinate.
  ///
  /// \param seg_idx The segment index to evaluate.
  /// \param road_s The road-local s-coordinate along the reference line.
  /// \return ReferenceLinePoint containing the evaluated coordinates.
  [[nodiscard]] auto Evaluate(std::uint32_t seg_idx, double road_s) const noexcept -> ReferenceLinePoint;

  /// Projects a point onto the reference line segment, finding the matching s-coordinate.
  ///
  /// \param seg_idx The segment index to project onto.
  /// \param px The target point's x coordinate.
  /// \param py The target point's y coordinate.
  /// \return The projected road-local s-coordinate along the reference line.
  [[nodiscard]] auto Project(std::uint32_t seg_idx, double px, double py) const noexcept -> double;

  /// Finds the segment index matching a given road and s-coordinate, leveraging context for coherence.
  ///
  /// \param road The compiled RoadId.
  /// \param s_coord The s-coordinate.
  /// \param ctx The query context for spatial coherence.
  /// \return The matched segment index.
  auto FindSegmentIndex(RoadId road, double s_coord, QueryContext& ctx) const noexcept -> std::uint32_t;

  /// Retrieves the segment index range (first segment index and count) for a given road.
  ///
  /// \param road The compiled RoadId.
  /// \return Pair of (first segment index, count).
  [[nodiscard]] auto GetRoadSegments(RoadId road) const noexcept -> std::pair<std::uint32_t, std::uint32_t>;

  /// Returns the start s-coordinate of the specified segment.
  ///
  /// \param seg_idx The segment index.
  /// \return The start s-coordinate.
  [[nodiscard]] auto GetSegmentSStart(std::uint32_t seg_idx) const noexcept -> double;

  /// Returns the length of the specified segment.
  ///
  /// \param seg_idx The segment index.
  /// \return The segment length.
  [[nodiscard]] auto GetSegmentLength(std::uint32_t seg_idx) const noexcept -> double;

  /// Computes the 2D Axis-Aligned Bounding Box (AABB) for a reference line segment.
  ///
  /// \param seg_idx The segment index.
  /// \param inflation The inflation distance to expand the bounding box.
  /// \return The computed AABB.
  [[nodiscard]] auto ComputeSegmentAabb(std::uint32_t seg_idx, double inflation) const noexcept -> Aabb;

  /// Returns the total number of segments compiled in the database.
  ///
  /// \return The count of segments.
  [[nodiscard]] auto TotalSegmentsCount() const noexcept -> std::size_t;

 private:
  // Encapsulated flat arrays (SoA)
  AlignedVector<double> s_offset_;
  AlignedVector<double> length_;
  AlignedVector<double> x_;
  AlignedVector<double> y_;
  AlignedVector<double> hdg_;
  std::vector<GeometryType> type_;
  std::vector<std::uint32_t> type_index_;
  AlignedVector<double> spiral_curv_start_;
  AlignedVector<double> spiral_curv_end_;
  AlignedVector<double> pp3_a_u_;
  AlignedVector<double> pp3_b_u_;
  AlignedVector<double> pp3_c_u_;
  AlignedVector<double> pp3_d_u_;
  AlignedVector<double> pp3_a_v_;
  AlignedVector<double> pp3_b_v_;
  AlignedVector<double> pp3_c_v_;
  AlignedVector<double> pp3_d_v_;
  std::vector<std::uint8_t> pp3_p_range_;

  // Curvature array (indexed by type_index_ for kArc segments)
  AlignedVector<double> arc_curvature_;

  // Road index mappings: maps road_idx -> segment range in the SoA
  std::vector<std::uint32_t> road_ref_line_first_idx_;
  std::vector<std::uint32_t> road_ref_line_count_;
};

}  // namespace strada::cpm
