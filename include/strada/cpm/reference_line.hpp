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

constexpr std::size_t kAlignmentBytes = 64;

template <typename T>
using AlignedVector = std::vector<T, AlignedAllocator<T, kAlignmentBytes>>;

enum class GeometryType : uint8_t { kLine, kArc, kSpiral, kPoly3, kParamPoly3 };

struct ReferenceLinePoint {
  double x{};
  double y{};
  double heading{};
};

struct Aabb {
  double min_x{};
  double min_y{};
  double max_x{};
  double max_y{};
};

class ReferenceLine {
 public:
  ReferenceLine() = default;

  // Static factory function to construct ReferenceLine from the AST
  static auto Build(const ast::AbstractSyntaxTree& map) -> ReferenceLine;

  // Move-only semantics
  ReferenceLine(const ReferenceLine&) = delete;
  auto operator=(const ReferenceLine&) -> ReferenceLine& = delete;
  ReferenceLine(ReferenceLine&&) noexcept = default;
  auto operator=(ReferenceLine&&) noexcept -> ReferenceLine& = default;
  ~ReferenceLine() = default;

  // Core math behaviors
  [[nodiscard]] auto Evaluate(uint32_t seg_idx, double global_s) const noexcept -> ReferenceLinePoint;
  [[nodiscard]] auto Project(uint32_t seg_idx, double px, double py) const noexcept -> double;  // Returns global_s
  auto FindSegmentIndex(RoadId road, double s_coord, QueryContext& ctx) const noexcept -> uint32_t;

  // Structural/Index queries
  [[nodiscard]] auto GetRoadSegments(RoadId road) const noexcept -> std::pair<uint32_t, uint32_t>;
  [[nodiscard]] auto GetSegmentSStart(uint32_t seg_idx) const noexcept -> double;
  [[nodiscard]] auto GetSegmentLength(uint32_t seg_idx) const noexcept -> double;
  [[nodiscard]] auto ComputeSegmentAabb(uint32_t seg_idx, double inflation) const noexcept -> Aabb;
  [[nodiscard]] auto TotalSegmentsCount() const noexcept -> std::size_t;

 private:
  // Encapsulated flat arrays (SoA)
  AlignedVector<double> s_offset_;
  AlignedVector<double> length_;
  AlignedVector<double> x_;
  AlignedVector<double> y_;
  AlignedVector<double> hdg_;
  std::vector<GeometryType> type_;
  std::vector<uint32_t> type_index_;
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
  std::vector<uint8_t> pp3_p_range_;

  // Curvature array (indexed by type_index_ for kArc segments)
  AlignedVector<double> arc_curvature_;

  // Road index mappings: maps road_idx -> segment range in the SoA
  std::vector<uint32_t> road_ref_line_first_idx_;
  std::vector<uint32_t> road_ref_line_count_;
};

}  // namespace strada::cpm
