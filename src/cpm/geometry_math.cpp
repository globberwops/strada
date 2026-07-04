#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numbers>
#include <strada/cpm/geometry_math.hpp>

namespace strada::cpm {

namespace {

constexpr auto kFn =
    std::array{0.49999988085884732562,  1.3511177791210715095,    1.3175407836168659241,   1.1861149300293854992,
               0.7709627298888346769,   0.4173874338787963957,    0.19044202705272903923,  0.06655998896627697537,
               0.022789258616785717418, 0.0040116689358507943804, 0.0012192036851249883877};

constexpr auto kFd = std::array{1.0,
                                2.7022305772400260215,
                                4.2059268151438492767,
                                4.5221882840107715516,
                                3.7240352281630359588,
                                2.4589286254678152943,
                                1.3125491629443702962,
                                0.5997685720120932908,
                                0.20907680750378849485,
                                0.07159621634657901433,
                                0.012602969513793714191,
                                0.0038302423512931250065};

constexpr auto kGn =
    std::array{0.50000014392706344801,  0.032346434925349128728,   0.17619325157863254363,   0.038606273170706486252,
               0.023693692309257725361, 0.007092018516845033662,   0.0012492123212412087428, 0.00044023040894778468486,
               -8.80266827476172521e-6, -1.4033554916580018648e-8, 2.3509221782155474353e-10};

constexpr auto kGd = std::array{1.0,
                                2.0646987497019598937,
                                2.9109311766948031235,
                                2.6561936751333032911,
                                2.0195563983177268073,
                                1.1167891129189363902,
                                0.57267874755973172715,
                                0.19408481169593070798,
                                0.07634808341431248904,
                                0.011573247407207865977,
                                0.0044099273693067311209,
                                -0.00009070958410429993314};

constexpr double kPi = std::numbers::pi;
constexpr double kPi2 = 0.5 * std::numbers::pi;
constexpr double k1SqrtPi = std::numbers::inv_sqrtpi;
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

constexpr auto kGaussPoints =
    std::array{-0.9061798459386640, -0.5384693101056831, 0.0, 0.5384693101056831, 0.9061798459386640};
constexpr auto kGaussWeights =
    std::array{0.2369268850561891, 0.4786286704993665, 0.5688888888888889, 0.4786286704993665, 0.2369268850561891};
constexpr int kNumGaussPoints = std::min(kGaussPoints.size(), kGaussWeights.size());

auto EvalXYaLarge(double a, double b) noexcept -> ClothoidResult {
  const double s = a > 0.0 ? 1.0 : -1.0;
  const double absa = std::abs(a);
  const double z = k1SqrtPi * std::sqrt(absa);
  const double ell = s * b * k1SqrtPi / std::sqrt(absa);
  const double g = -0.5 * s * (b * b) / absa;
  const double cg = std::cos(g) / z;
  const double sg = std::sin(g) / z;

  auto [cl, sl] = FresnelCS(ell);
  auto [cz, sz] = FresnelCS(ell + z);

  const double dc0 = cz - cl;
  const double ds0 = sz - sl;

  const double x_val = (cg * dc0) - (s * sg * ds0);
  const double y_val = (sg * dc0) + (s * cg * ds0);
  return ClothoidResult{.x = x_val, .y = y_val};
}

}  // namespace

auto FresnelCS(double y) noexcept -> FresnelResult {
  constexpr double kEps = 1e-15;
  const double x = y > 0.0 ? y : -y;
  double c = 0.0;
  double s = 0.0;

  if (x < 1.0) {
    double twofn = kNaN;
    double fact = kNaN;
    double denterm = kNaN;
    double numterm = kNaN;
    double sum = kNaN;
    double term = kNaN;

    const double s_val = kPi2 * (x * x);
    const double t_val = -s_val * s_val;

    twofn = 0.0;
    fact = 1.0;
    denterm = 1.0;
    numterm = 1.0;
    sum = 1.0;
    do {
      twofn += 2.0;
      fact *= twofn * (twofn - 1.0);
      denterm += 4.0;
      numterm *= t_val;
      term = numterm / (fact * denterm);
      sum += term;
    } while (std::abs(term) > kEps * std::abs(sum));

    c = x * sum;

    twofn = 1.0;
    fact = 1.0;
    denterm = 3.0;
    numterm = 1.0;
    sum = 1.0 / 3.0;
    do {
      twofn += 2.0;
      fact *= twofn * (twofn - 1.0);
      denterm += 4.0;
      numterm *= t_val;
      term = numterm / (fact * denterm);
      sum += term;
    } while (std::abs(term) > kEps * std::abs(sum));

    s = kPi2 * sum * (x * x * x);

  } else if (x < 6.0) {
    double sumn = 0.0;
    double sumd = kFd[11];
    for (int k = 10; k >= 0; --k) {
      sumn = kFn[k] + (x * sumn);
      sumd = kFd[k] + (x * sumd);
    }
    const double f_val = sumn / sumd;

    sumn = 0.0;
    sumd = kGd[11];
    for (int k = 10; k >= 0; --k) {
      sumn = kGn[k] + (x * sumn);
      sumd = kGd[k] + (x * sumd);
    }
    const double g_val = sumn / sumd;

    const double u_val = kPi2 * (x * x);
    const double sin_u = std::sin(u_val);
    const double cos_u = std::cos(u_val);
    c = 0.5 + (f_val * sin_u) - (g_val * cos_u);
    s = 0.5 - (f_val * cos_u) - (g_val * sin_u);

  } else {
    double absterm = kNaN;
    const double s_val = kPi * x * x;
    const double t_val = -1.0 / (s_val * s_val);

    double numterm = -1.0;
    double term = 1.0;
    double sum = 1.0;
    double oldterm = 1.0;
    const double eps10 = 0.1 * kEps;

    do {
      numterm += 4.0;
      term *= numterm * (numterm - 2.0) * t_val;
      sum += term;
      absterm = std::abs(term);
      if (oldterm < absterm) {
        break;
      }
      oldterm = absterm;
    } while (absterm > eps10 * std::abs(sum));

    const double f_val = sum / (kPi * x);

    numterm = -1.0;
    term = 1.0;
    sum = 1.0;
    oldterm = 1.0;

    do {
      numterm += 4.0;
      term *= numterm * (numterm + 2.0) * t_val;
      sum += term;
      absterm = std::abs(term);
      if (oldterm < absterm) {
        break;
      }
      oldterm = absterm;
    } while (absterm > eps10 * std::abs(sum));

    double g_val = kPi * x;
    g_val = sum / (g_val * g_val * x);

    const double u_val = kPi2 * (x * x);
    const double sin_u = std::sin(u_val);
    const double cos_u = std::cos(u_val);
    c = 0.5 + (f_val * sin_u) - (g_val * cos_u);
    s = 0.5 - (f_val * cos_u) - (g_val * sin_u);
  }

  if (y < 0.0) {
    c = -c;
    s = -s;
  }

  return FresnelResult{.c = c, .s = s};
}

auto EvaluateClothoidIntegrals(double param_a, double param_b) noexcept -> ClothoidResult {
  if (std::abs(param_a) < 1e-4) {
    double x0 = 0.0;
    double y0 = 0.0;
    double x2 = 0.0;
    double y2 = 0.0;
    const double abs_b = std::abs(param_b);
    if (abs_b < 0.1) {
      const double b2 = param_b * param_b;
      const double b4 = b2 * b2;
      const double b6 = b4 * b2;
      x0 = 1.0 - (b2 / 6.0) + (b4 / 120.0) - (b6 / 5040.0);
      y0 = (param_b / 2.0) - ((param_b * b2) / 24.0) + ((param_b * b4) / 720.0) - ((param_b * b6) / 40320.0);
      x2 = (1.0 / 3.0) - (b2 / 10.0) + (b4 / 168.0) - (b6 / 6480.0);
      y2 = (param_b / 4.0) - ((param_b * b2) / 36.0) + ((param_b * b4) / 960.0) - ((param_b * b6) / 50400.0);
    } else {
      const double sin_b = std::sin(param_b);
      const double cos_b = std::cos(param_b);
      const double inv_b = 1.0 / param_b;
      const double inv_b2 = inv_b * inv_b;
      const double inv_b3 = inv_b2 * inv_b;
      x0 = sin_b * inv_b;
      y0 = (1.0 - cos_b) * inv_b;
      x2 = (sin_b * inv_b) + (2.0 * (cos_b * inv_b2)) - (2.0 * (sin_b * inv_b3));
      y2 = (-(cos_b * inv_b)) + (2.0 * (sin_b * inv_b2)) - (2.0 * ((1.0 - cos_b) * inv_b3));
    }
    const double x_val = x0 - (0.5 * (param_a * y2));
    const double y_val = y0 + (0.5 * (param_a * x2));
    return {.x = x_val, .y = y_val};
  }
  return EvalXYaLarge(param_a, param_b);
}

auto IntegrateArcLength(double u, double b, double c, double d) noexcept -> double {
  double sum = 0.0;
  const double half_u = 0.5 * u;
  for (int i = 0; i < kNumGaussPoints; ++i) {
    const double sigma = half_u * (kGaussPoints[i] + 1.0);
    const double v_prime = b + (2.0 * c * sigma) + (3.0 * d * sigma * sigma);
    sum += kGaussWeights[i] * std::sqrt(1.0 + (v_prime * v_prime));
  }
  return half_u * sum;
}

auto SolveUForS(double s_target, double length, double b_u, double b, double c, double d) noexcept -> double {
  if (s_target <= 0.0) {
    return 0.0;
  }
  double u_val = s_target * b_u;
  constexpr double kTol = 1e-12;
  constexpr int kMaxIter = 100;
  for (int iter = 0; iter < kMaxIter; ++iter) {
    const double s_val = IntegrateArcLength(u_val, b, c, d);
    const double v_prime = b + (2.0 * c * u_val) + (3.0 * d * u_val * u_val);
    const double f_val = s_val - s_target;
    const double f_prime = std::sqrt(1.0 + (v_prime * v_prime));
    if (std::abs(f_prime) < 1e-12) {
      break;
    }
    const double diff = f_val / f_prime;
    u_val -= diff;
    if (std::abs(f_val) < kTol) {
      break;
    }
  }
  return u_val;
}

auto ConvertPoly3ToParamPoly3(double length, double a, double b, double c, double d) noexcept -> ast::ParamPoly3 {
  ast::ParamPoly3 param;
  param.p_range = ast::PRange::kArcLength;

  if (length <= 0.0) {
    param.a_u = 0.0;
    param.b_u = 0.0;
    param.c_u = 0.0;
    param.d_u = 0.0;
    param.a_v = a;
    param.b_v = 0.0;
    param.c_v = 0.0;
    param.d_v = 0.0;
    return param;
  }

  const double den = std::sqrt(1.0 + (b * b));
  const double b_u = 1.0 / den;
  const double b_v = b / den;

  const double u1 = SolveUForS(0.5 * length, length, b_u, b, c, d);
  const double u2 = SolveUForS(length, length, b_u, b, c, d);

  const double v1 = a + (u1 * (b + u1 * (c + d * u1)));
  const double v2 = a + (u2 * (b + u2 * (c + d * u2)));

  param.a_u = 0.0;
  param.b_u = b_u;
  param.c_u = (8.0 * u1 - u2 - 3.0 * b_u * length) / (length * length);
  param.d_u = (2.0 * u2 - 8.0 * u1 + 2.0 * b_u * length) / (length * length * length);

  param.a_v = a;
  param.b_v = b_v;
  param.c_v = (8.0 * v1 - v2 - 7.0 * a - 3.0 * b_v * length) / (length * length);
  param.d_v = (2.0 * v2 - 8.0 * v1 + 6.0 * a + 2.0 * b_v * length) / (length * length * length);

  return param;
}

}  // namespace strada::cpm
