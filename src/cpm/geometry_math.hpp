#pragma once

#include <strada/ast/abstract_syntax_tree.hpp>

namespace strada::cpm {

/// Results of evaluating the Fresnel sine and cosine integrals.
struct FresnelResult {
  double c{};  ///< Cosine integral value C(y)
  double s{};  ///< Sine integral value S(y)
};

/// Computes the Fresnel Cosine C(y) and Sine S(y) integrals.
///
/// \param y The input evaluation point.
/// \return A FresnelResult containing c and s.
auto FresnelCS(double y) noexcept -> FresnelResult;

/// Results of evaluating the clothoid coordinate offsets.
struct ClothoidResult {
  double x{};  ///< Tangent offset coordinate
  double y{};  ///< Normal offset coordinate
};

/// Evaluates clothoid (Euler spiral) integral coordinate offsets.
///
/// \param param_a The scaling rate of curvature change parameter.
/// \param param_b The initial curvature parameter.
/// \return A ClothoidResult containing the evaluated coordinates (x, y).
auto EvaluateClothoidIntegrals(double param_a, double param_b) noexcept -> ClothoidResult;

/// Integrates the arc length of a cubic polynomial curve.
auto IntegrateArcLength(double u, double b, double c, double d) noexcept -> double;

/// Solves for the polynomial parameter u that corresponds to a target arc length s.
auto SolveUForS(double s_target, double length, double b_u, double b, double c, double d) noexcept -> double;

/// Converts a standard cubic polynomial curve into its arc-length parameterized form.
auto ConvertPoly3ToParamPoly3(double length, double a, double b, double c, double d) noexcept -> ast::ParamPoly3;

}  // namespace strada::cpm
