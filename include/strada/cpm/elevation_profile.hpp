// Copyright 2026 Google LLC
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <strada/ast/abstract_syntax_tree.hpp>
#include <strada/cpm/aligned_allocator.hpp>
#include <strada/cpm/reference_line.hpp>
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

/// Represents evaluated vertical properties of a road at a given (s, t) point.
struct VerticalProfile {
  double elevation{};     ///< The absolute elevation z of the reference line.
  double pitch{};         ///< The pitch angle along the reference line.
  double natural_roll{};  ///< The bank/superelevation angle of the road surface.
  double roll_total{};    ///< The total roll angle (superelevation plus shape profile bank).
  double shape_height{};  ///< The lateral shape height offset.
};

/// Compiled representation of the elevation, superelevation, and shape profiles of a road network.
class ElevationProfile {
 public:
  /// Default constructor.
  ElevationProfile() = default;

  /// Destructor.
  ~ElevationProfile() = default;

  // Move-only semantics
  ElevationProfile(const ElevationProfile&) = delete;
  auto operator=(const ElevationProfile&) -> ElevationProfile& = delete;
  ElevationProfile(ElevationProfile&&) noexcept = default;
  auto operator=(ElevationProfile&&) noexcept -> ElevationProfile& = default;

  /// Builds the ElevationProfile database from the map abstract syntax tree.
  ///
  /// \param map The map AST.
  /// \return Compiled ElevationProfile.
  static auto Build(const ast::AbstractSyntaxTree& map) -> ElevationProfile;

  /// Evaluates the vertical profile properties at the specified s and t coordinates on a road.
  ///
  /// \param road The compiled RoadId.
  /// \param s The s-station coordinate along the road's reference line.
  /// \param t The lateral t-offset from the road's reference line.
  /// \return The evaluated vertical profile.
  [[nodiscard]] auto Evaluate(RoadId road, double s, double t) const noexcept -> VerticalProfile;

  /// Evaluates shape height offset at the specified s and t coordinates.
  ///
  /// \param road The compiled RoadId.
  /// \param s The s-station coordinate.
  /// \param t The lateral t-offset.
  /// \return The shape height.
  [[nodiscard]] auto EvaluateShapeHeight(RoadId road, double s, double t) const noexcept -> double;

  /// Evaluates shape t-gradient at the specified s and t coordinates.
  ///
  /// \param road The compiled RoadId.
  /// \param s The s-station coordinate.
  /// \param t The lateral t-offset.
  /// \return The shape t-gradient.
  [[nodiscard]] auto EvaluateShapeTGradient(RoadId road, double s, double t) const noexcept -> double;

 private:
  ElevationSoA elevation_;
  ShapesSoA shapes_;
  PolynomialsSoA polynomials_;
  std::vector<uint8_t> road_has_css_;
};

}  // namespace strada::cpm
