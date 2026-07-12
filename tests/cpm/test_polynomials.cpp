#include <gtest/gtest.h>

#include <strada/cpm/polynomials.hpp>
#include <vector>

TEST(PolynomialsTest, CompileAndEvaluateSingleSegment) {
  // Arrange
  strada::cpm::Polynomials polynomials;
  std::vector<strada::ast::Coefficient> coeffs = {{10.0, 1.5, 2.0, 0.5, 0.1}};

  // Act
  auto [first_idx, count] = polynomials.Compile(coeffs);

  double val = polynomials.Evaluate(first_idx, count, 12.0);
  double deriv = polynomials.EvaluateDerivative(first_idx, count, 12.0);

  // Assert
  EXPECT_EQ(first_idx, 0);
  EXPECT_EQ(count, 1);
  EXPECT_DOUBLE_EQ(val, 8.3);
  EXPECT_DOUBLE_EQ(deriv, 5.2);
}

TEST(PolynomialsTest, CompileAndEvaluateMultipleSegments) {
  // Arrange
  strada::cpm::Polynomials polynomials;
  std::vector<strada::ast::Coefficient> coeffs = {
      {0.0, 1.0, 0.0, 0.0, 0.0},  // s in [0, 10), constant 1.0
      {10.0, 2.0, 1.0, 0.0, 0.0}  // s in [10, 20), linear 2.0 + 1.0 * ds
  };

  // Act
  auto [first_idx, count] = polynomials.Compile(coeffs);

  // Assert basic evaluation in segments
  EXPECT_DOUBLE_EQ(polynomials.Evaluate(first_idx, count, 5.0), 1.0);
  EXPECT_DOUBLE_EQ(polynomials.Evaluate(first_idx, count, 15.0), 7.0);

  // Assert boundaries and clamping
  EXPECT_DOUBLE_EQ(polynomials.Evaluate(first_idx, count, -5.0), 1.0);   // Clamps to first segment
  EXPECT_DOUBLE_EQ(polynomials.Evaluate(first_idx, count, 25.0), 17.0);  // Clamps to last segment

  // Assert derivatives
  EXPECT_DOUBLE_EQ(polynomials.EvaluateDerivative(first_idx, count, 5.0), 0.0);
  EXPECT_DOUBLE_EQ(polynomials.EvaluateDerivative(first_idx, count, 15.0), 1.0);
}

TEST(PolynomialsTest, EvaluateEmptyPolynomials) {
  // Arrange
  strada::cpm::Polynomials polynomials;

  // Act & Assert
  EXPECT_DOUBLE_EQ(polynomials.Evaluate(0, 0, 5.0), 0.0);
  EXPECT_DOUBLE_EQ(polynomials.EvaluateDerivative(0, 0, 5.0), 0.0);
}
