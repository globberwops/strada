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
  const double kS = a > 0.0 ? 1.0 : -1.0;
  const double kAbsa = std::abs(a);
  const double kZ = k1SqrtPi * std::sqrt(kAbsa);
  const double kEll = kS * b * k1SqrtPi / std::sqrt(kAbsa);
  const double kG = -0.5 * kS * (b * b) / kAbsa;
  const double kCg = std::cos(kG) / kZ;
  const double kSg = std::sin(kG) / kZ;

  auto [cl, sl] = FresnelCS(kEll);
  auto [cz, sz] = FresnelCS(kEll + kZ);

  const double kDc0 = cz - cl;
  const double kDs0 = sz - sl;

  const double kXVal = (kCg * kDc0) - (kS * kSg * kDs0);
  const double kYVal = (kSg * kDc0) + (kS * kCg * kDs0);
  return ClothoidResult{.x = kXVal, .y = kYVal};
}

}  // namespace

auto FresnelCS(double y) noexcept -> FresnelResult {
  constexpr double kEps = 1e-15;
  const double kX = y > 0.0 ? y : -y;
  double c = 0.0;
  double s = 0.0;

  if (kX < 1.0) {
    double twofn = kNaN;
    double fact = kNaN;
    double denterm = kNaN;
    double numterm = kNaN;
    double sum = kNaN;
    double term = kNaN;

    const double kSVal = kPi2 * (kX * kX);
    const double kTVal = -kSVal * kSVal;

    twofn = 0.0;
    fact = 1.0;
    denterm = 1.0;
    numterm = 1.0;
    sum = 1.0;
    do {
      twofn += 2.0;
      fact *= twofn * (twofn - 1.0);
      denterm += 4.0;
      numterm *= kTVal;
      term = numterm / (fact * denterm);
      sum += term;
    } while (std::abs(term) > kEps * std::abs(sum));

    c = kX * sum;

    twofn = 1.0;
    fact = 1.0;
    denterm = 3.0;
    numterm = 1.0;
    sum = 1.0 / 3.0;
    do {
      twofn += 2.0;
      fact *= twofn * (twofn - 1.0);
      denterm += 4.0;
      numterm *= kTVal;
      term = numterm / (fact * denterm);
      sum += term;
    } while (std::abs(term) > kEps * std::abs(sum));

    s = kPi2 * sum * (kX * kX * kX);

  } else if (kX < 6.0) {
    double sumn = 0.0;
    double sumd = kFd[11];
    for (int k = 10; k >= 0; --k) {
      sumn = kFn[k] + (kX * sumn);
      sumd = kFd[k] + (kX * sumd);
    }
    const double kFVal = sumn / sumd;

    sumn = 0.0;
    sumd = kGd[11];
    for (int k = 10; k >= 0; --k) {
      sumn = kGn[k] + (kX * sumn);
      sumd = kGd[k] + (kX * sumd);
    }
    const double kGVal = sumn / sumd;

    const double kUVal = kPi2 * (kX * kX);
    const double kSinU = std::sin(kUVal);
    const double kCosU = std::cos(kUVal);
    c = 0.5 + (kFVal * kSinU) - (kGVal * kCosU);
    s = 0.5 - (kFVal * kCosU) - (kGVal * kSinU);

  } else {
    double absterm = kNaN;
    const double kSVal = kPi * kX * kX;
    const double kTVal = -1.0 / (kSVal * kSVal);

    double numterm = -1.0;
    double term = 1.0;
    double sum = 1.0;
    double oldterm = 1.0;
    const double kEps10 = 0.1 * kEps;

    do {
      numterm += 4.0;
      term *= numterm * (numterm - 2.0) * kTVal;
      sum += term;
      absterm = std::abs(term);
      if (oldterm < absterm) {
        break;
      }
      oldterm = absterm;
    } while (absterm > kEps10 * std::abs(sum));

    const double kFVal = sum / (kPi * kX);

    numterm = -1.0;
    term = 1.0;
    sum = 1.0;
    oldterm = 1.0;

    do {
      numterm += 4.0;
      term *= numterm * (numterm + 2.0) * kTVal;
      sum += term;
      absterm = std::abs(term);
      if (oldterm < absterm) {
        break;
      }
      oldterm = absterm;
    } while (absterm > kEps10 * std::abs(sum));

    double g_val = kPi * kX;
    g_val = sum / (g_val * g_val * kX);

    const double kUVal = kPi2 * (kX * kX);
    const double kSinU = std::sin(kUVal);
    const double kCosU = std::cos(kUVal);
    c = 0.5 + (kFVal * kSinU) - (g_val * kCosU);
    s = 0.5 - (kFVal * kCosU) - (g_val * kSinU);
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
    const double kAbsB = std::abs(param_b);
    if (kAbsB < 0.1) {
      const double kB2 = param_b * param_b;
      const double kB4 = kB2 * kB2;
      const double kB6 = kB4 * kB2;
      x0 = 1.0 - (kB2 / 6.0) + (kB4 / 120.0) - (kB6 / 5040.0);
      y0 = (param_b / 2.0) - ((param_b * kB2) / 24.0) + ((param_b * kB4) / 720.0) - ((param_b * kB6) / 40320.0);
      x2 = (1.0 / 3.0) - (kB2 / 10.0) + (kB4 / 168.0) - (kB6 / 6480.0);
      y2 = (param_b / 4.0) - ((param_b * kB2) / 36.0) + ((param_b * kB4) / 960.0) - ((param_b * kB6) / 50400.0);
    } else {
      const double kSinB = std::sin(param_b);
      const double kCosB = std::cos(param_b);
      const double kInvB = 1.0 / param_b;
      const double kInvB2 = kInvB * kInvB;
      const double kInvB3 = kInvB2 * kInvB;
      x0 = kSinB * kInvB;
      y0 = (1.0 - kCosB) * kInvB;
      x2 = (kSinB * kInvB) + (2.0 * (kCosB * kInvB2)) - (2.0 * (kSinB * kInvB3));
      y2 = (-(kCosB * kInvB)) + (2.0 * (kSinB * kInvB2)) - (2.0 * ((1.0 - kCosB) * kInvB3));
    }
    const double kXVal = x0 - (0.5 * (param_a * y2));
    const double kYVal = y0 + (0.5 * (param_a * x2));
    return {.x = kXVal, .y = kYVal};
  }
  return EvalXYaLarge(param_a, param_b);
}

auto IntegrateArcLength(double u, double b, double c, double d) noexcept -> double {
  double sum = 0.0;
  const double kHalfU = 0.5 * u;
  for (int i = 0; i < kNumGaussPoints; ++i) {
    const double kSigma = kHalfU * (kGaussPoints[i] + 1.0);
    const double kVPrime = b + (2.0 * c * kSigma) + (3.0 * d * kSigma * kSigma);
    sum += kGaussWeights[i] * std::sqrt(1.0 + (kVPrime * kVPrime));
  }
  return kHalfU * sum;
}

auto SolveUForS(double s_target, double length, double b_u, double b, double c, double d) noexcept -> double {
  if (s_target <= 0.0) {
    return 0.0;
  }
  double u_val = s_target * b_u;
  constexpr double kTol = 1e-12;
  constexpr int kMaxIter = 100;
  for (int iter = 0; iter < kMaxIter; ++iter) {
    const double kSVal = IntegrateArcLength(u_val, b, c, d);
    const double kVPrime = b + (2.0 * c * u_val) + (3.0 * d * u_val * u_val);
    const double kFVal = kSVal - s_target;
    const double kFPrime = std::sqrt(1.0 + (kVPrime * kVPrime));
    if (std::abs(kFPrime) < 1e-12) {
      break;
    }
    const double kDiff = kFVal / kFPrime;
    u_val -= kDiff;
    if (std::abs(kFVal) < kTol) {
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

  const double kDen = std::sqrt(1.0 + (b * b));
  const double kBU = 1.0 / kDen;
  const double kBV = b / kDen;

  const double kU1 = SolveUForS(0.5 * length, length, kBU, b, c, d);
  const double kU2 = SolveUForS(length, length, kBU, b, c, d);

  const double kV1 = a + (kU1 * (b + kU1 * (c + d * kU1)));
  const double kV2 = a + (kU2 * (b + kU2 * (c + d * kU2)));

  param.a_u = 0.0;
  param.b_u = kBU;
  param.c_u = (8.0 * kU1 - kU2 - 3.0 * kBU * length) / (length * length);
  param.d_u = (2.0 * kU2 - 8.0 * kU1 + 2.0 * kBU * length) / (length * length * length);

  param.a_v = a;
  param.b_v = kBV;
  param.c_v = (8.0 * kV1 - kV2 - 7.0 * a - 3.0 * kBV * length) / (length * length);
  param.d_v = (2.0 * kV2 - 8.0 * kV1 + 6.0 * a + 2.0 * kBV * length) / (length * length * length);

  return param;
}

}  // namespace strada::cpm
