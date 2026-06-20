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
///
/// Computes the arc length of the curve v(u) = a + b*u + c*u^2 + d*u^3 from 0 to u.
///
/// \param u The upper integration limit parameter.
/// \param b The linear coefficient of the polynomial curve.
/// \param c The quadratic coefficient of the polynomial curve.
/// \param d The cubic coefficient of the polynomial curve.
/// \return The integrated arc-length coordinate s.
auto IntegrateArcLength(double u, double b, double c, double d) noexcept -> double;

/// Solves for the polynomial parameter u that corresponds to a target arc length s.
///
/// Uses Newton-Raphson iteration to find the parameter u such that the integrated arc length equals s_target.
///
/// \param s_target The target arc length coordinate.
/// \param length The total length of the segment.
/// \param b_u The initial scaling parameter derivative du/ds at u = 0.
/// \param b The linear coefficient of the polynomial curve.
/// \param c The quadratic coefficient of the polynomial curve.
/// \param d The cubic coefficient of the polynomial curve.
/// \return The solved parameter u.
auto SolveUForS(double s_target, double length, double b_u, double b, double c, double d) noexcept -> double;

/// Converts a standard cubic polynomial curve into its arc-length parameterized form.
///
/// Fits a parametric cubic polynomial curve u(p) and v(p) where the parameter p is the arc-length.
///
/// \param length The longitudinal length of the curve.
/// \param a The constant coefficient of the standard cubic polynomial.
/// \param b The linear coefficient of the standard cubic polynomial.
/// \param c The quadratic coefficient of the standard cubic polynomial.
/// \param d The cubic coefficient of the standard cubic polynomial.
/// \return The converted ParamPoly3 structure.
auto ConvertPoly3ToParamPoly3(double length, double a, double b, double c, double d) noexcept -> ast::ParamPoly3;

}  // namespace strada::cpm
