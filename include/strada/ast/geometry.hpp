#ifndef STRADA_AST_GEOMETRY_HPP_
#define STRADA_AST_GEOMETRY_HPP_

#include <cstdint>
#include <variant>

namespace strada::ast {

struct Line {
  // Line has no extra parameters in the OpenDRIVE XML other than the geometry header.
};

struct Spiral {
  double curv_start_{};
  double curv_end_{};
};

struct Arc {
  double curvature_{};
};

struct Poly3 {
  double a_{};
  double b_{};
  double c_{};
  double d_{};
};

enum class PRange : std::uint8_t { Normalized, ArcLength };

struct ParamPoly3 {
  double a_u_{};
  double b_u_{};
  double c_u_{};
  double d_u_{};
  double a_v_{};
  double b_v_{};
  double c_v_{};
  double d_v_{};
  PRange p_range_ = PRange::Normalized;
};

using GeometryShape = std::variant<Line, Spiral, Arc, Poly3, ParamPoly3>;

struct GeometryRecord {
  double s_{};
  double x_{};
  double y_{};
  double hdg_{};
  double length_{};
  GeometryShape shape_;
};

}  // namespace strada::ast

#endif  // STRADA_AST_GEOMETRY_HPP_
