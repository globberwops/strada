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

constexpr std::size_t K_ALIGNMENT_BYTES = 64;

template <typename T>
using AlignedVector = std::vector<T, AlignedAllocator<T, K_ALIGNMENT_BYTES>>;

enum class GeometryType : uint8_t { kLine, kArc, kSpiral, kPoly3, kParamPoly3 };

struct ReferenceLineSoA {
  AlignedVector<double> s_offset;
  AlignedVector<double> length;
  AlignedVector<double> x;
  AlignedVector<double> y;
  AlignedVector<double> hdg;
  std::vector<GeometryType> type;
  std::vector<uint32_t> type_index;
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
  auto InertialToRoad(InertialPosition position, QueryContext& ctx) const noexcept -> std::optional<RoadPose>;
  auto InertialToLane(InertialPosition position, QueryContext& ctx) const noexcept -> std::optional<LanePose>;
  auto RoadToLane(RoadPose pose, QueryContext& ctx) const noexcept -> std::optional<LanePose>;
  auto LaneToRoad(LanePose pose, QueryContext& ctx) const noexcept -> RoadPose;

  // Inspection: noexcept, stateless.
  // NOLINTNEXTLINE(readability-identifier-naming)
  [[nodiscard]] auto road_count() const noexcept -> std::size_t;

  // NOLINTNEXTLINE(readability-identifier-naming)
  [[nodiscard]] auto road_id_from_string(std::string_view original_id) const noexcept -> std::optional<RoadId>;

  // NOLINTNEXTLINE(readability-identifier-naming)
  [[nodiscard]] auto original_road_id(RoadId road_id) const noexcept -> std::string_view;

  // NOLINTNEXTLINE(readability-identifier-naming)
  [[nodiscard]] auto road_length(RoadId road_id) const noexcept -> double;

  // NOLINTNEXTLINE(readability-identifier-naming)
  [[nodiscard]] auto lane_count() const noexcept -> std::size_t;

  // NOLINTNEXTLINE(readability-identifier-naming)
  [[nodiscard]] auto lane_road(LaneId lane_id) const noexcept -> RoadId;

  // NOLINTNEXTLINE(readability-identifier-naming)
  [[nodiscard]] auto original_lane_id(LaneId lane_id) const noexcept -> int;

  // NOLINTNEXTLINE(readability-identifier-naming)
  [[nodiscard]] auto lane_width(LaneId lane_id, double s_coord) const noexcept -> double;

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
};

auto BuildCompiledPhysicsModel(const ast::AbstractSyntaxTree& map) -> CompiledPhysicsModel;

}  // namespace strada::cpm
