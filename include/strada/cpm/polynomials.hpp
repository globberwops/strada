#pragma once

#include <cstdint>
#include <strada/ast/abstract_syntax_tree.hpp>
#include <strada/cpm/reference_line.hpp>
#include <utility>
#include <vector>

namespace strada::cpm {

/// Consolidated database of road/lane profile polynomials.
class Polynomials {
 public:
  Polynomials() = default;
  ~Polynomials() = default;

  Polynomials(const Polynomials&) = delete;
  auto operator=(const Polynomials&) -> Polynomials& = delete;
  Polynomials(Polynomials&&) noexcept = default;
  auto operator=(Polynomials&&) noexcept -> Polynomials& = default;

  /// Compiles a sequence of coefficients and stores them.
  /// \return Pair of (first_idx, count).
  auto Compile(const std::vector<ast::Coefficient>& coeffs) -> std::pair<std::uint32_t, std::uint32_t>;

  /// Evaluates the polynomial at a given s-coordinate.
  [[nodiscard]] auto Evaluate(std::uint32_t first_idx, std::uint32_t count, double s_coord) const noexcept -> double;

  /// Evaluates the derivative of the polynomial at a given s-coordinate.
  [[nodiscard]] auto EvaluateDerivative(std::uint32_t first_idx, std::uint32_t count, double s_coord) const noexcept
      -> double;

  [[nodiscard]] auto Empty() const noexcept -> bool { return s_start_.empty(); }

  void Clear() noexcept {
    s_start_.clear();
    a_.clear();
    b_.clear();
    c_.clear();
    d_.clear();
  }

 private:
  AlignedVector<double> s_start_;
  AlignedVector<double> a_;
  AlignedVector<double> b_;
  AlignedVector<double> c_;
  AlignedVector<double> d_;
};

}  // namespace strada::cpm
