#include <gtest/gtest.h>

#include <cmath>

#include "../../src/cpm/geometry_math.hpp"

namespace strada::cpm {

TEST(GeometryMathTest, FresnelCS_SmallArgument) {
  // Arrange
  double y = 0.5;

  // Act
  auto result = FresnelCS(y);

  // Assert
  EXPECT_NEAR(result.c, 0.49234422587144638, 1e-9);
  EXPECT_NEAR(result.s, 0.06473243285999927, 1e-9);
}

TEST(GeometryMathTest, FresnelCS_MediumArgument) {
  // Act
  auto result = FresnelCS(3.0);

  // Assert (values verified from algorithm execution)
  EXPECT_NEAR(result.c, 0.605720789297686, 1e-9);
  EXPECT_NEAR(result.s, 0.49631299896737496, 1e-9);
}

TEST(GeometryMathTest, FresnelCS_LargeArgument) {
  // Act
  auto result = FresnelCS(10.0);

  // Assert (values verified from algorithm execution)
  EXPECT_NEAR(result.c, 0.49989869420551575, 1e-9);
  EXPECT_NEAR(result.s, 0.46816997858488224, 1e-9);
}

TEST(GeometryMathTest, FresnelCS_Symmetry) {
  // Act
  auto pos_res = FresnelCS(0.5);
  auto neg_res = FresnelCS(-0.5);

  // Assert odd symmetry: C(-y) = -C(y), S(-y) = -S(y)
  EXPECT_DOUBLE_EQ(neg_res.c, -pos_res.c);
  EXPECT_DOUBLE_EQ(neg_res.s, -pos_res.s);
}

TEST(GeometryMathTest, EvaluateClothoidIntegrals_SmallA) {
  // Arrange
  double param_a = 0.0;
  double param_b = 0.05;

  // Act
  auto result = EvaluateClothoidIntegrals(param_a, param_b);

  // Assert (values verified from algorithm execution)
  EXPECT_NEAR(result.x, 0.99958338541356651, 1e-9);
  EXPECT_NEAR(result.y, 0.024994792100675071, 1e-9);
}

TEST(GeometryMathTest, EvaluateClothoidIntegrals_LargeA) {
  // Arrange
  double param_a = 1.0;
  double param_b = 0.5;

  // Act
  auto result = EvaluateClothoidIntegrals(param_a, param_b);

  // Assert (values verified from algorithm execution)
  EXPECT_NEAR(result.x, 0.87677470886535303, 1e-9);
  EXPECT_NEAR(result.y, 0.38654634658885301, 1e-9);
}

TEST(GeometryMathTest, IntegrateArcLength) {
  // Arrange
  double u = 5.0;
  double b = 0.1;
  double c = -0.02;
  double d = 0.005;

  // Act
  double length = IntegrateArcLength(u, b, c, d);

  // Assert (values verified from algorithm execution)
  EXPECT_NEAR(length, 5.0469743185022775, 1e-9);
}

TEST(GeometryMathTest, SolveUForS) {
  // Arrange
  double s_target = 4.0;
  double length = 5.0;
  double b = 0.1;
  double c = -0.02;
  double d = 0.005;
  double den = std::sqrt(1.0 + (b * b));
  double b_u = 1.0 / den;

  // Act
  double u_sol = SolveUForS(s_target, length, b_u, b, c, d);

  // Assert: integrating over the solved u should yield s_target
  double integrated_s = IntegrateArcLength(u_sol, b, c, d);
  EXPECT_NEAR(integrated_s, s_target, 1e-9);
}

TEST(GeometryMathTest, ConvertPoly3ToParamPoly3) {
  // Arrange
  double length = 10.0;
  double a = 1.5;
  double b = -0.2;
  double c = 0.05;
  double d = -0.01;

  // Act
  auto param = ConvertPoly3ToParamPoly3(length, a, b, c, d);

  // Assert (values verified from algorithm execution)
  EXPECT_EQ(param.p_range, ast::PRange::kArcLength);
  EXPECT_NEAR(param.a_u, 0.0, 1e-9);
  EXPECT_NEAR(param.b_u, 0.9805806756909202, 1e-9);
  EXPECT_NEAR(param.c_u, 0.011874853022157232, 1e-9);
  EXPECT_NEAR(param.d_u, -0.0024637047402425376, 1e-9);
  EXPECT_NEAR(param.a_v, 1.5, 1e-9);
  EXPECT_NEAR(param.b_v, -0.19611613513818404, 1e-9);
  EXPECT_NEAR(param.c_v, 0.025375282504920584, 1e-9);
  EXPECT_NEAR(param.d_v, -0.0048501953734593475, 1e-9);
}

}  // namespace strada::cpm
