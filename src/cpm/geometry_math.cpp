#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numbers>
#include <strada/cpm/geometry_math.hpp>

namespace strada::cpm {

namespace {

constexpr auto kSn = std::array{-2.99181919401019853726E3, 7.08840045257738576863E5,   -6.29741486205862506537E7,
                                2.54890880573376359104E9,  -4.42979518059697779103E10, 3.18016297876567817986E11};

constexpr auto kSd = std::array{2.81376268889994315696E2, 4.55847810806532581675E4,  5.17343888770096400730E6,
                                4.19320245898111231129E8, 2.24411795645340920940E10, 6.07366389490084639049E11};

constexpr auto kCn = std::array{-4.98843114573573548651E-8, 9.50428062829859605134E-6,  -6.45191435683965050962E-4,
                                1.88843319396703850064E-2,  -2.05525900955013891793E-1, 9.99999999999999998822E-1};

constexpr auto kCd = std::array{3.99982968972495980367E-12, 9.15439215774657478799E-10, 1.25001862479598821474E-7,
                                1.22262789024179030997E-5,  8.68029542941784300606E-4,  4.12142090722199792936E-2,
                                1.00000000000000000118E0};

constexpr auto kFn = std::array{4.21543555043677546506E-1,  1.43407919780758885261E-1,  1.15220955073585758835E-2,
                                3.45017939782574027900E-4,  4.63613749287867322088E-6,  3.05568983790257605827E-8,
                                1.02304514164907233465E-10, 1.72010743268161828879E-13, 1.34283276233062758925E-16,
                                3.76329711269987889006E-20};

constexpr auto kFd = std::array{7.51586398353378947175E-1,  1.16888925859191382142E-1,  6.44051526508858611005E-3,
                                1.55934409164153020873E-4,  1.84627567348930545870E-6,  1.12699224763999035261E-8,
                                3.60140029589371370404E-11, 5.88754533621578410010E-14, 4.52001434074129701496E-17,
                                1.25443237090011264384E-20};

constexpr auto kGn = std::array{5.04442073643383265887E-1,  1.97102833525523411709E-1,  1.87648584092575249293E-2,
                                6.84079380915393090172E-4,  1.15138826111884280931E-5,  9.82852443688422223854E-8,
                                4.45344415861750144738E-10, 1.08268041139020870318E-12, 1.37555460633261799868E-15,
                                8.36354435630677421531E-19, 1.86958710162783235106E-22};

constexpr auto kGd = std::array{1.47495759925128324529E0,   3.37748989120019970451E-1,  2.53603741420338795122E-2,
                                8.14679107184306179049E-4,  1.27545075667729118702E-5,  1.04314589657571990585E-7,
                                4.60680728146520428211E-10, 1.10273215066240270757E-12, 1.38796531259578871258E-15,
                                8.39158816283118707363E-19, 1.86958710162783236342E-22};

constexpr double kPi{std::numbers::pi};
constexpr double kPi2{0.5 * std::numbers::pi};
constexpr double k1SqrtPi{std::numbers::inv_sqrtpi};
constexpr double kNaN{std::numeric_limits<double>::quiet_NaN()};

constexpr auto kGaussPoints =
    std::array{-0.9061798459386640, -0.5384693101056831, 0.0, 0.5384693101056831, 0.9061798459386640};
constexpr auto kGaussWeights =
    std::array{0.2369268850561891, 0.4786286704993665, 0.5688888888888889, 0.4786286704993665, 0.2369268850561891};
constexpr int kNumGaussPoints = std::min(kGaussPoints.size(), kGaussWeights.size());

template <std::size_t N>
auto EvaluatePolynomial(double x, const std::array<double, N>& coef) noexcept -> double {
  double ans{coef[0]};
  for (std::size_t i{1}; i < N; ++i) {
    ans = (ans * x) + coef[i];
  }
  return ans;
}

template <std::size_t N>
auto EvaluateMonicPolynomial(double x, const std::array<double, N>& coef) noexcept -> double {
  double ans{x + coef[0]};
  for (std::size_t i{1}; i < N; ++i) {
    ans = (ans * x) + coef[i];
  }
  return ans;
}

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
  const double x{std::abs(y)};
  const double x2{x * x};
  double cc{0.0};
  double ss{0.0};

  if (x2 < 2.5625) {
    const double t{x2 * x2};
    ss = x * x2 * EvaluatePolynomial(t, kSn) / EvaluateMonicPolynomial(t, kSd);
    cc = x * EvaluatePolynomial(t, kCn) / EvaluatePolynomial(t, kCd);
  } else if (x > 36974.0) {
    cc = 0.5;
    ss = 0.5;
  } else {
    const double t_val{kPi * x2};
    const double u{1.0 / (t_val * t_val)};
    const double t_inv{1.0 / t_val};
    const double f{1.0 - u * EvaluatePolynomial(u, kFn) / EvaluateMonicPolynomial(u, kFd)};
    const double g{t_inv * EvaluatePolynomial(u, kGn) / EvaluateMonicPolynomial(u, kGd)};

    const double angle{kPi2 * x2};
    const double sin_val{std::sin(angle)};
    const double cos_val{std::cos(angle)};
    const double t_denom{kPi * x};
    cc = 0.5 + ((f * sin_val) - (g * cos_val)) / t_denom;
    ss = 0.5 - ((f * cos_val) + (g * sin_val)) / t_denom;
  }

  if (y < 0.0) {
    cc = -cc;
    ss = -ss;
  }

  return FresnelResult{.c = cc, .s = ss};
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
  constexpr double tol = 1e-12;
  constexpr int max_iter = 100;
  for (int iter = 0; iter < max_iter; ++iter) {
    const double s_val = IntegrateArcLength(u_val, b, c, d);
    const double v_prime = b + (2.0 * c * u_val) + (3.0 * d * u_val * u_val);
    const double f_val = s_val - s_target;
    const double f_prime = std::sqrt(1.0 + (v_prime * v_prime));
    if (std::abs(f_prime) < 1e-12) {
      break;
    }
    const double diff = f_val / f_prime;
    u_val -= diff;
    if (std::abs(f_val) < tol) {
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
  const double bu = 1.0 / den;
  const double bv = b / den;

  const double u1 = SolveUForS(0.5 * length, length, bu, b, c, d);
  const double u2 = SolveUForS(length, length, bu, b, c, d);

  const double v1 = a + (u1 * (b + u1 * (c + d * u1)));
  const double v2 = a + (u2 * (b + u2 * (c + d * u2)));

  param.a_u = 0.0;
  param.b_u = bu;
  param.c_u = (8.0 * u1 - u2 - 3.0 * bu * length) / (length * length);
  param.d_u = (2.0 * u2 - 8.0 * u1 + 2.0 * bu * length) / (length * length * length);

  param.a_v = a;
  param.b_v = bv;
  param.c_v = (8.0 * v1 - v2 - 7.0 * a - 3.0 * bv * length) / (length * length);
  param.d_v = (2.0 * v2 - 8.0 * v1 + 6.0 * a + 2.0 * bv * length) / (length * length * length);

  return param;
}

}  // namespace strada::cpm
