// SPDX-License-Identifier: BSL-1.0

#pragma once

#include <cstdint>
#include <optional>
#include <vector>

namespace strada::ast {

/// Defines a road elevation profile polynomial segment.
struct Elevation {
  double s{};  ///< Start s-station of the elevation segment.
  double a{};  ///< Constant coefficient (a).
  double b{};  ///< Linear coefficient (b).
  double c{};  ///< Quadratic coefficient (c).
  double d{};  ///< Cubic coefficient (d).
};

/// Defines a road superelevation (banking) profile polynomial segment.
struct Superelevation {
  double s{};  ///< Start s-station of the superelevation segment.
  double a{};  ///< Constant coefficient (a).
  double b{};  ///< Linear coefficient (b).
  double c{};  ///< Quadratic coefficient (c).
  double d{};  ///< Cubic coefficient (d).
};

/// Defines a road lateral shape profile segment at a given t-offset.
struct Shape {
  double s{};  ///< Start s-station of the shape segment.
  double t{};  ///< Lateral t-offset of the shape.
  double a{};  ///< Constant coefficient (a).
  double b{};  ///< Linear coefficient (b).
  double c{};  ///< Quadratic coefficient (c).
  double d{};  ///< Cubic coefficient (d).
};

/// Generic polynomial coefficient container.
struct Coefficient {
  double s{};  ///< Start s-station of the coefficient segment.
  double a{};  ///< Constant coefficient (a).
  double b{};  ///< Linear coefficient (b).
  double c{};  ///< Quadratic coefficient (c).
  double d{};  ///< Cubic coefficient (d).
};

/// Mode of height evaluation for cross-section surface strips.
enum class StripMode : std::uint8_t {
  kIndependent = 0,  ///< maps to XML string "independent"
  kRelative          ///< maps to XML string "relative"
};

/// Represents a single strip in a cross-section surface profile.
struct CrossSectionSurfaceStrip {
  int id{};                                 ///< Strip ID (positive on left, negative on right).
  StripMode mode{StripMode::kIndependent};  ///< Mode of height evaluation.
  std::vector<Coefficient> constant;        ///< Polynomial constant (c0) coefficients.
  std::vector<Coefficient> linear;          ///< Polynomial linear (c1) coefficients.
  std::vector<Coefficient> quadratic;       ///< Polynomial quadratic (c2) coefficients.
  std::vector<Coefficient> cubic;           ///< Polynomial cubic (c3) coefficients.
  std::vector<Coefficient> width;           ///< Width polynomial coefficients.
};

/// Defines a road's cross-section surface (lateral elevation strips).
struct CrossSectionSurface {
  std::vector<Coefficient> t_offset;             ///< Offset of the reference line.
  std::vector<CrossSectionSurfaceStrip> strips;  ///< All surface strips sorted by ID.
};

/// Container for the road elevation profile.
struct ElevationProfile {
  std::vector<Elevation> elevations;  ///< Contiguous elevation polynomial segments.
};

/// Container for road lateral profiles (superelevations, shapes, cross-section surfaces).
struct LateralProfile {
  std::vector<Superelevation> superelevations;               ///< Contiguous superelevation polynomial segments.
  std::vector<Shape> shapes;                                 ///< Contiguous lateral shapes.
  std::optional<CrossSectionSurface> cross_section_surface;  ///< Optional cross-section surface definition.
};

}  // namespace strada::ast
