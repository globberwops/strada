// SPDX-License-Identifier: BSL-1.0

#pragma once

#include <cstdint>
#include <variant>

namespace strada::ast {

/// Plan-view straight line segment geometry.
struct Line {
  // Line has no extra parameters in the OpenDRIVE XML other than the geometry header.
};

/// Plan-view spiral (clothoid) curve geometry.
struct Spiral {
  double curv_start{};  ///< Curvature at the start of the spiral segment.
  double curv_end{};    ///< Curvature at the end of the spiral segment.
};

/// Plan-view circular arc segment geometry.
struct Arc {
  double curvature{};  ///< Constant curvature of the arc (1/radius).
};

/// Plan-view cubic polynomial curve geometry.
struct Poly3 {
  double a{};  ///< Constant coefficient (a).
  double b{};  ///< Linear coefficient (b).
  double c{};  ///< Quadratic coefficient (c).
  double d{};  ///< Cubic coefficient (d).
};

/// Definition range of parameter p for parametric cubic polynomials.
enum class PRange : std::uint8_t {
  kNormalized = 0,  ///< Parameter p is in [0, 1].
  kArcLength        ///< Parameter p is in [0, length].
};

/// Plan-view parametric cubic polynomial curve geometry.
struct ParamPoly3 {
  double a_u{};                         ///< Constant coefficient for u(p).
  double b_u{};                         ///< Linear coefficient for u(p).
  double c_u{};                         ///< Quadratic coefficient for u(p).
  double d_u{};                         ///< Cubic coefficient for u(p).
  double a_v{};                         ///< Constant coefficient for v(p).
  double b_v{};                         ///< Linear coefficient for v(p).
  double c_v{};                         ///< Quadratic coefficient for v(p).
  double d_v{};                         ///< Cubic coefficient for v(p).
  PRange p_range{PRange::kNormalized};  ///< Parameter definition range.
};

/// Represents any plan-view geometry segment type.
using GeometryShape = std::variant<Line, Spiral, Arc, Poly3, ParamPoly3>;

/// Container for a plan-view reference line segment header and its geometric shape.
struct GeometryRecord {
  double s{};           ///< Start s-station along the reference line.
  double x{};           ///< Start x coordinate in the global frame.
  double y{};           ///< Start y coordinate in the global frame.
  double hdg{};         ///< Heading angle at start s-station.
  double length{};      ///< Length of this geometry segment.
  GeometryShape shape;  ///< The concrete geometry type parameterization.
};

}  // namespace strada::ast
