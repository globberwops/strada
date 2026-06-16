#pragma once

#include <cstddef>
#include <optional>
#include <strada/ast/opendrive.hpp>
#include <strada/cpm/coordinate.hpp>
#include <strada/cpm/ids.hpp>
#include <strada/cpm/query_context.hpp>
#include <string_view>
#include <vector>

namespace strada::cpm {

// Flat SoA structures for Cross Section Surface per ADR 0005
struct PolynomialsSoA {
  std::vector<double> s_start;
  std::vector<double> a;
  std::vector<double> b;
  std::vector<double> c;
  std::vector<double> d;
};

struct StripsSoA {
  std::vector<int32_t> strip_id;
  std::vector<bool> is_relative;
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
  CompiledPhysicsModel& operator=(const CompiledPhysicsModel&) = delete;
  CompiledPhysicsModel(CompiledPhysicsModel&&) noexcept = default;
  CompiledPhysicsModel& operator=(CompiledPhysicsModel&&) noexcept = default;

  // Hot-path queries: noexcept
  auto RoadToInertial(RoadPose, QueryContext&) const noexcept -> InertialPose;

  // Inspection APIs
  auto road_count() const noexcept -> std::size_t;
  auto road_id_from_string(std::string_view) const noexcept -> std::optional<RoadId>;
  auto original_road_id(RoadId) const noexcept -> std::string_view;

 private:
  friend auto BuildCompiledPhysicsModel(const ast::OpenDrive& map) -> CompiledPhysicsModel;

  // Road ID mapping tables
  std::vector<std::string> road_string_ids_;

  // Cross section surface flat SoA structures
  PolynomialsSoA polynomials_;
  StripsSoA strips_;
  RoadCrossSectionSurfaceSoA road_css_;
};

auto BuildCompiledPhysicsModel(const ast::OpenDrive& map) -> CompiledPhysicsModel;

}  // namespace strada::cpm
