#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numbers>
#include <span>
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

constexpr double kPi = std::numbers::pi;
constexpr double kPi2 = 0.5 * std::numbers::pi;
constexpr double k1SqrtPi = std::numbers::inv_sqrtpi;
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

constexpr auto kGaussPoints =
    std::array{-0.9061798459386640, -0.5384693101056831, 0.0, 0.5384693101056831, 0.9061798459386640};
constexpr auto kGaussWeights =
    std::array{0.2369268850561891, 0.4786286704993665, 0.5688888888888889, 0.4786286704993665, 0.2369268850561891};
constexpr int kNumGaussPoints = std::min(kGaussPoints.size(), kGaussWeights.size());

template <std::size_t N>
constexpr auto EvaluatePolynomial(const double x, const std::span<const double, N> coef) noexcept -> double {
  auto ans = coef[0];
  for (auto i = std::size_t{1}; i < N; ++i) {
    ans = (ans * x) + coef[i];
  }
  return ans;
}

template <std::size_t N>
constexpr auto EvaluatePolynomial(const double x, const std::array<double, N>& coef) noexcept -> double {
  return EvaluatePolynomial(x, std::span<const double, N>{coef});
}

template <std::size_t N>
constexpr auto EvaluateMonicPolynomial(const double x, const std::span<const double, N> coef) noexcept -> double {
  auto ans = x + coef[0];
  for (auto i = std::size_t{1}; i < N; ++i) {
    ans = (ans * x) + coef[i];
  }
  return ans;
}

template <std::size_t N>
constexpr auto EvaluateMonicPolynomial(const double x, const std::array<double, N>& coef) noexcept -> double {
  return EvaluateMonicPolynomial(x, std::span<const double, N>{coef});
}

auto EvalXYaLarge(double a, double b) noexcept -> ClothoidResult {
  const double s = a > 0.0 ? 1.0 : -1.0;
  const double absa = std::abs(a);
  const double z = k1SqrtPi * std::sqrt(absa);
  const double ell = s * b * k1SqrtPi / std::sqrt(absa);
  const double gamma = -0.5 * s * (b * b) / absa;
  const double cos_g = std::cos(gamma) / z;
  const double sin_g = std::sin(gamma) / z;

  auto [cl, sl] = FresnelCS(ell);
  auto [cz, sz] = FresnelCS(ell + z);

  const double dc0 = cz - cl;
  const double ds0 = sz - sl;

  const double x_val = (cos_g * dc0) - (s * sin_g * ds0);
  const double y_val = (sin_g * dc0) + (s * cos_g * ds0);
  return ClothoidResult{.x = x_val, .y = y_val};
}

}  // namespace

auto FresnelCS(double y) noexcept -> FresnelResult {
  const auto x = std::abs(y);
  const auto x_sq = x * x;
  auto fresnel_c = 0.0;
  auto fresnel_s = 0.0;

  constexpr double k_fresnel_threshold_sq = 2.5625;
  constexpr double k_fresnel_asymptotic_cutoff = 36974.0;

  if (x_sq < k_fresnel_threshold_sq) {
    const auto t = x_sq * x_sq;
    fresnel_s = x * x_sq * EvaluatePolynomial(t, kSn) / EvaluateMonicPolynomial(t, kSd);
    fresnel_c = x * EvaluatePolynomial(t, kCn) / EvaluatePolynomial(t, kCd);
  } else if (x > k_fresnel_asymptotic_cutoff) {
    fresnel_c = 0.5;
    fresnel_s = 0.5;
  } else {
    const auto t_val = kPi * x_sq;
    const auto u = 1.0 / (t_val * t_val);
    const auto t_inv = 1.0 / t_val;
    const auto f = 1.0 - (u * EvaluatePolynomial(u, kFn) / EvaluateMonicPolynomial(u, kFd));
    const auto g_val = t_inv * EvaluatePolynomial(u, kGn) / EvaluateMonicPolynomial(u, kGd);

    const auto angle = kPi2 * x_sq;
    const auto sin_val = std::sin(angle);
    const auto cos_val = std::cos(angle);
    const auto t_denom = kPi * x;
    fresnel_c = 0.5 + (((f * sin_val) - (g_val * cos_val)) / t_denom);
    fresnel_s = 0.5 - (((f * cos_val) + (g_val * sin_val)) / t_denom);
  }

  if (y < 0.0) {
    fresnel_c = -fresnel_c;
    fresnel_s = -fresnel_s;
  }

  return FresnelResult{.c = fresnel_c, .s = fresnel_s};
}

auto EvaluateClothoidIntegrals(double param_a, double param_b) noexcept -> ClothoidResult {
  constexpr double k_small_a_threshold = 1e-4;
  constexpr double k_small_b_threshold = 0.1;
  constexpr double k_fact_3 = 6.0;
  constexpr double k_fact_4 = 24.0;
  constexpr double k_fact_5 = 120.0;
  constexpr double k_fact_6 = 720.0;
  constexpr double k_fact_7 = 5040.0;
  constexpr double k_fact_8 = 40320.0;

  constexpr double k_x2_coef_1 = 3.0;
  constexpr double k_x2_coef_2 = 10.0;
  constexpr double k_x2_coef_3 = 168.0;
  constexpr double k_x2_coef_4 = 6480.0;

  constexpr double k_y2_coef_1 = 4.0;
  constexpr double k_y2_coef_2 = 36.0;
  constexpr double k_y2_coef_3 = 960.0;
  constexpr double k_y2_coef_4 = 50400.0;

  if (std::abs(param_a) < k_small_a_threshold) {
    double x0_val = 0.0;
    double y0_val = 0.0;
    double x2_val = 0.0;
    double y2_val = 0.0;
    const double abs_b = std::abs(param_b);
    if (abs_b < k_small_b_threshold) {
      const double b_sq = param_b * param_b;
      const double b_quad = b_sq * b_sq;
      const double b_hex = b_quad * b_sq;
      x0_val = 1.0 - (b_sq / k_fact_3) + (b_quad / k_fact_5) - (b_hex / k_fact_7);
      y0_val = (param_b / 2.0) - ((param_b * b_sq) / k_fact_4) + ((param_b * b_quad) / k_fact_6) -
               ((param_b * b_hex) / k_fact_8);
      x2_val = (1.0 / k_x2_coef_1) - (b_sq / k_x2_coef_2) + (b_quad / k_x2_coef_3) - (b_hex / k_x2_coef_4);
      y2_val = (param_b / k_y2_coef_1) - ((param_b * b_sq) / k_y2_coef_2) + ((param_b * b_quad) / k_y2_coef_3) -
               ((param_b * b_hex) / k_y2_coef_4);
    } else {
      const double sin_b = std::sin(param_b);
      const double cos_b = std::cos(param_b);
      const double inv_b = 1.0 / param_b;
      const double inv_b2 = inv_b * inv_b;
      const double inv_b3 = inv_b2 * inv_b;
      x0_val = sin_b * inv_b;
      y0_val = (1.0 - cos_b) * inv_b;
      x2_val = (sin_b * inv_b) + (2.0 * (cos_b * inv_b2)) - (2.0 * (sin_b * inv_b3));
      y2_val = (-(cos_b * inv_b)) + (2.0 * (sin_b * inv_b2)) - (2.0 * ((1.0 - cos_b) * inv_b3));
    }
    const double x_val = x0_val - (0.5 * (param_a * y2_val));
    const double y_val = y0_val + (0.5 * (param_a * x2_val));
    return {.x = x_val, .y = y_val};
  }
  return EvalXYaLarge(param_a, param_b);
}

auto IntegrateArcLength(double u, double b, double c, double d) noexcept -> double {
  double sum = 0.0;
  const double half_u = 0.5 * u;
  for (auto i = std::size_t{0}; i < static_cast<std::size_t>(kNumGaussPoints); ++i) {
    const double sigma = half_u * (kGaussPoints.at(i) + 1.0);
    const double v_prime = b + (2.0 * c * sigma) + (3.0 * d * sigma * sigma);
    sum += kGaussWeights.at(i) * std::sqrt(1.0 + (v_prime * v_prime));
  }
  return half_u * sum;
}

auto SolveUForS(double s_target, double b_u, double b, double c, double d) noexcept -> double {
  if (s_target <= 0.0) {
    return 0.0;
  }
  double u_val = s_target * b_u;
  constexpr auto k_tolerance = 1e-12;
  constexpr auto max_iter = 100;
  for (int iter = 0; iter < max_iter; ++iter) {
    const double s_val = IntegrateArcLength(u_val, b, c, d);
    const double v_prime = b + (2.0 * c * u_val) + (3.0 * d * u_val * u_val);
    const double f_val = s_val - s_target;
    const double f_prime = std::sqrt(1.0 + (v_prime * v_prime));
    if (std::abs(f_prime) < k_tolerance) {
      break;
    }
    const double diff = f_val / f_prime;
    u_val -= diff;
    if (std::abs(f_val) < k_tolerance) {
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
  const double b_u_scale = 1.0 / den;
  const double b_v_scale = b / den;

  const double u_mid = SolveUForS(0.5 * length, b_u_scale, b, c, d);
  const double u_end = SolveUForS(length, b_u_scale, b, c, d);

  const double v_mid = a + (u_mid * (b + u_mid * (c + d * u_mid)));
  const double v_end = a + (u_end * (b + u_end * (c + d * u_end)));

  constexpr double k_fit_coef_3 = 3.0;
  constexpr double k_fit_coef_6 = 6.0;
  constexpr double k_fit_coef_7 = 7.0;
  constexpr double k_fit_coef_8 = 8.0;

  param.a_u = 0.0;
  param.b_u = b_u_scale;
  param.c_u = (k_fit_coef_8 * u_mid - u_end - k_fit_coef_3 * b_u_scale * length) / (length * length);
  param.d_u = (2.0 * u_end - k_fit_coef_8 * u_mid + 2.0 * b_u_scale * length) / (length * length * length);

  param.a_v = a;
  param.b_v = b_v_scale;
  param.c_v = (k_fit_coef_8 * v_mid - v_end - k_fit_coef_7 * a - k_fit_coef_3 * b_v_scale * length) / (length * length);
  param.d_v =
      (2.0 * v_end - k_fit_coef_8 * v_mid + k_fit_coef_6 * a + 2.0 * b_v_scale * length) / (length * length * length);
  return param;
}

}  // namespace strada::cpm
