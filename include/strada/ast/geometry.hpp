#pragma once

#include <cstdint>
#include <variant>

namespace strada::ast {

struct Line {
  // Line has no extra parameters in the OpenDRIVE XML other than the geometry header.
};

struct Spiral {
  double curv_start{};
  double curv_end{};
};

struct Arc {
  double curvature{};
};

struct Poly3 {
  double a{};
  double b{};
  double c{};
  double d{};
};

enum class PRange : std::uint8_t { Normalized, ArcLength };

struct ParamPoly3 {
  double a_u{};
  double b_u{};
  double c_u{};
  double d_u{};
  double a_v{};
  double b_v{};
  double c_v{};
  double d_v{};
  PRange p_range = PRange::Normalized;
};

using GeometryShape = std::variant<Line, Spiral, Arc, Poly3, ParamPoly3>;

struct GeometryRecord {
  double s{};
  double x{};
  double y{};
  double hdg{};
  double length{};
  GeometryShape shape;
};

}  // namespace strada::ast
