// SPDX-License-Identifier: BSL-1.0

#include <strada/cpm/polynomials.hpp>

namespace strada::cpm {

auto Polynomials::Compile(const std::vector<ast::Coefficient>& coeffs) -> std::pair<std::uint32_t, std::uint32_t> {
  auto first_idx = static_cast<std::uint32_t>(s_start_.size());
  auto count = static_cast<std::uint32_t>(coeffs.size());
  for (const auto& coeff : coeffs) {
    s_start_.push_back(coeff.s);
    a_.push_back(coeff.a);
    b_.push_back(coeff.b);
    c_.push_back(coeff.c);
    d_.push_back(coeff.d);
  }
  return {first_idx, count};
}

auto Polynomials::Evaluate(std::uint32_t first_idx, std::uint32_t count, double s_coord) const noexcept -> double {
  if (count == 0) {
    return 0.0;
  }
  std::uint32_t active_idx = first_idx;
  for (std::uint32_t i = 0; i < count; ++i) {
    const std::uint32_t kIdx = first_idx + i;
    if (s_coord >= s_start_[kIdx]) {
      active_idx = kIdx;
    } else {
      break;
    }
  }
  const double kDsVal = s_coord - s_start_[active_idx];
  return a_[active_idx] + (b_[active_idx] * kDsVal) + (c_[active_idx] * kDsVal * kDsVal) +
         (d_[active_idx] * kDsVal * kDsVal * kDsVal);
}

auto Polynomials::EvaluateDerivative(std::uint32_t first_idx, std::uint32_t count, double s_coord) const noexcept
    -> double {
  if (count == 0) {
    return 0.0;
  }
  std::uint32_t active_idx = first_idx;
  for (std::uint32_t i = 0; i < count; ++i) {
    const std::uint32_t kIdx = first_idx + i;
    if (s_coord >= s_start_[kIdx]) {
      active_idx = kIdx;
    } else {
      break;
    }
  }
  const double kDsVal = s_coord - s_start_[active_idx];
  return b_[active_idx] + (2.0 * c_[active_idx] * kDsVal) + (3.0 * d_[active_idx] * kDsVal * kDsVal);
}

}  // namespace strada::cpm
