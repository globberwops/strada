// SPDX-License-Identifier: BSL-1.0

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <strada/ast/abstract_syntax_tree.hpp>
#include <strada/cpm/elevation_profile.hpp>
#include <strada/cpm/ids.hpp>
#include <vector>

namespace strada::cpm {

namespace {

auto EvaluatePolynomial(const PolynomialsSoA& poly, std::uint32_t first_idx, std::uint32_t count,
                        double s_coord) noexcept -> double {
  if (count == 0) {
    return 0.0;
  }
  std::uint32_t active_idx = first_idx;
  for (std::uint32_t i = 0; i < count; ++i) {
    const std::uint32_t idx = first_idx + i;
    if (s_coord >= poly.s_start[idx]) {
      active_idx = idx;
    } else {
      break;
    }
  }
  const double ds_val = s_coord - poly.s_start[active_idx];
  return poly.a[active_idx] + (poly.b[active_idx] * ds_val) + (poly.c[active_idx] * ds_val * ds_val) +
         (poly.d[active_idx] * ds_val * ds_val * ds_val);
}

void CompileCoefficients(const std::vector<ast::Coefficient>& coeffs, PolynomialsSoA& dest, std::uint32_t& first_idx,
                         std::uint32_t& count) {
  first_idx = static_cast<std::uint32_t>(dest.s_start.size());
  count = static_cast<std::uint32_t>(coeffs.size());
  for (const auto& coeff : coeffs) {
    dest.s_start.push_back(coeff.s);
    dest.a.push_back(coeff.a);
    dest.b.push_back(coeff.b);
    dest.c.push_back(coeff.c);
    dest.d.push_back(coeff.d);
  }
}

struct ShapeGroup {
  double s{};
  std::uint32_t first_idx{};
  std::uint32_t count{};
};

void FindShapeGroups(const ShapesSoA& shapes, std::uint32_t first_idx, std::uint32_t count, double s_coord,
                     std::optional<ShapeGroup>& g1, std::optional<ShapeGroup>& g2) noexcept {
  g1 = std::nullopt;
  g2 = std::nullopt;
  if (count == 0) {
    return;
  }

  double max_s_le = -std::numeric_limits<double>::max();
  double min_s_ge = std::numeric_limits<double>::max();
  bool found_le = false;
  bool found_ge = false;

  for (std::uint32_t i = 0; i < count; ++i) {
    const double s_val = shapes.s[first_idx + i];
    if (s_val <= s_coord) {
      if (!found_le || s_val > max_s_le) {
        max_s_le = s_val;
        found_le = true;
      }
    }
    if (s_val >= s_coord) {
      if (!found_ge || s_val < min_s_ge) {
        min_s_ge = s_val;
        found_ge = true;
      }
    }
  }

  if (found_le) {
    ShapeGroup group;
    group.s = max_s_le;
    group.first_idx = 0;
    group.count = 0;
    bool in_group = false;
    for (std::uint32_t i = 0; i < count; ++i) {
      const std::uint32_t idx = first_idx + i;
      if (shapes.s[idx] == max_s_le) {
        if (!in_group) {
          group.first_idx = idx;
          in_group = true;
        }
        group.count++;
      }
    }
    g1 = group;
  }

  if (found_ge) {
    ShapeGroup group;
    group.s = min_s_ge;
    group.first_idx = 0;
    group.count = 0;
    bool in_group = false;
    for (std::uint32_t i = 0; i < count; ++i) {
      const std::uint32_t idx = first_idx + i;
      if (shapes.s[idx] == min_s_ge) {
        if (!in_group) {
          group.first_idx = idx;
          in_group = true;
        }
        group.count++;
      }
    }
    g2 = group;
  }
}

auto EvaluateGroupHeight(const ShapesSoA& shapes, const ShapeGroup& group, double t_coord) noexcept -> double {
  if (group.count == 0) {
    return 0.0;
  }
  std::uint32_t active_idx = group.first_idx;
  for (std::uint32_t i = 0; i < group.count; ++i) {
    const std::uint32_t idx = group.first_idx + i;
    if (t_coord >= shapes.t[idx]) {
      active_idx = idx;
    } else {
      break;
    }
  }
  const double dt = t_coord - shapes.t[active_idx];
  return shapes.a[active_idx] + (dt * (shapes.b[active_idx] + dt * (shapes.c[active_idx] + dt * shapes.d[active_idx])));
}

auto EvaluateGroupTGradient(const ShapesSoA& shapes, const ShapeGroup& group, double t_coord) noexcept -> double {
  if (group.count == 0) {
    return 0.0;
  }
  std::uint32_t active_idx = group.first_idx;
  for (std::uint32_t i = 0; i < group.count; ++i) {
    const std::uint32_t idx = group.first_idx + i;
    if (t_coord >= shapes.t[idx]) {
      active_idx = idx;
    } else {
      break;
    }
  }
  const double dt = t_coord - shapes.t[active_idx];
  return shapes.b[active_idx] + (dt * (2.0 * shapes.c[active_idx] + dt * 3.0 * shapes.d[active_idx]));
}

}  // namespace

auto ElevationProfile::Build(const ast::AbstractSyntaxTree& map) -> ElevationProfile {
  ElevationProfile profile;

  for (const auto& road : map.roads) {
    // Elevation profile compilation
    {
      auto first_idx = 0U;
      auto count = 0U;
      std::vector<ast::Coefficient> coeffs;
      coeffs.reserve(road.elevation_profile.elevations.size());
      for (const auto& elev : road.elevation_profile.elevations) {
        coeffs.push_back({elev.s, elev.a, elev.b, elev.c, elev.d});
      }
      CompileCoefficients(coeffs, profile.polynomials_, first_idx, count);
      profile.elevation_.road_elevation_first_idx.push_back(first_idx);
      profile.elevation_.road_elevation_count.push_back(count);
    }

    // Superelevation profile compilation
    {
      auto first_idx = 0U;
      auto count = 0U;
      std::vector<ast::Coefficient> coeffs;
      coeffs.reserve(road.lateral_profile.superelevations.size());
      for (const auto& super : road.lateral_profile.superelevations) {
        coeffs.push_back({super.s, super.a, super.b, super.c, super.d});
      }
      CompileCoefficients(coeffs, profile.polynomials_, first_idx, count);
      profile.elevation_.road_superelevation_first_idx.push_back(first_idx);
      profile.elevation_.road_superelevation_count.push_back(count);
    }

    // Shape profile compilation
    {
      auto first_idx = static_cast<std::uint32_t>(profile.shapes_.s.size());
      auto count = static_cast<std::uint32_t>(road.lateral_profile.shapes.size());
      for (const auto& shape : road.lateral_profile.shapes) {
        profile.shapes_.s.push_back(shape.s);
        profile.shapes_.t.push_back(shape.t);
        profile.shapes_.a.push_back(shape.a);
        profile.shapes_.b.push_back(shape.b);
        profile.shapes_.c.push_back(shape.c);
        profile.shapes_.d.push_back(shape.d);
      }
      profile.shapes_.road_shape_first_idx.push_back(first_idx);
      profile.shapes_.road_shape_count.push_back(count);
    }

    // Record whether road has cross section surface strips
    bool has_css = road.lateral_profile.cross_section_surface.has_value();
    profile.road_has_css_.push_back(static_cast<std::uint8_t>(has_css));
  }

  return profile;
}

auto ElevationProfile::Evaluate(RoadId road, double s, double t) const noexcept -> VerticalProfile {
  auto road_idx = static_cast<std::uint32_t>(road);
  if (road_idx >= road_has_css_.size()) {
    return VerticalProfile{};
  }

  double elev = 0.0;
  double natural_pitch = 0.0;
  double natural_roll = 0.0;

  const bool has_css = (road_has_css_[road_idx] != 0U);

  elev = EvaluatePolynomial(polynomials_, elevation_.road_elevation_first_idx[road_idx],
                            elevation_.road_elevation_count[road_idx], s);

  double d_elev = 0.0;
  const std::uint32_t elev_first_idx = elevation_.road_elevation_first_idx[road_idx];
  const std::uint32_t elev_count = elevation_.road_elevation_count[road_idx];
  if (elev_count > 0) {
    std::uint32_t active_idx = elev_first_idx;
    for (std::uint32_t i = 0; i < elev_count; ++i) {
      const std::uint32_t idx = elev_first_idx + i;
      if (s >= polynomials_.s_start[idx]) {
        active_idx = idx;
      } else {
        break;
      }
    }
    const double ds_poly = s - polynomials_.s_start[active_idx];
    d_elev = polynomials_.b[active_idx] + (2.0 * polynomials_.c[active_idx] * ds_poly) +
             (3.0 * polynomials_.d[active_idx] * ds_poly * ds_poly);
  }
  natural_pitch = std::atan(d_elev);

  if (!has_css) {
    natural_roll = EvaluatePolynomial(polynomials_, elevation_.road_superelevation_first_idx[road_idx],
                                      elevation_.road_superelevation_count[road_idx], s);
  }

  const double shape_h = EvaluateShapeHeight(road, s, t);
  const double shape_grad = EvaluateShapeTGradient(road, s, t);

  VerticalProfile profile;
  profile.elevation = elev;
  profile.pitch = natural_pitch;
  profile.natural_roll = natural_roll;
  profile.roll_total = natural_roll + std::atan(shape_grad);
  profile.shape_height = shape_h;

  return profile;
}

auto ElevationProfile::EvaluateShapeHeight(RoadId road, double s, double t) const noexcept -> double {
  auto road_idx = static_cast<std::uint32_t>(road);
  if (road_idx >= shapes_.road_shape_first_idx.size()) {
    return 0.0;
  }

  const std::uint32_t first_idx = shapes_.road_shape_first_idx[road_idx];
  const std::uint32_t count = shapes_.road_shape_count[road_idx];

  std::optional<ShapeGroup> g1;
  std::optional<ShapeGroup> g2;
  FindShapeGroups(shapes_, first_idx, count, s, g1, g2);

  if (!g1.has_value() && !g2.has_value()) {
    return 0.0;
  }
  if (g1.has_value() && !g2.has_value()) {
    return EvaluateGroupHeight(shapes_, *g1, t);
  }
  if (!g1.has_value() && g2.has_value()) {
    return EvaluateGroupHeight(shapes_, *g2, t);
  }

  const double h1 = EvaluateGroupHeight(shapes_, *g1, t);
  if (g1->s == g2->s) {
    return h1;
  }
  const double h2 = EvaluateGroupHeight(shapes_, *g2, t);
  const double f = (s - g1->s) / (g2->s - g1->s);
  return ((1.0 - f) * h1) + (f * h2);
}

auto ElevationProfile::EvaluateShapeTGradient(RoadId road, double s, double t) const noexcept -> double {
  auto road_idx = static_cast<std::uint32_t>(road);
  if (road_idx >= shapes_.road_shape_first_idx.size()) {
    return 0.0;
  }

  const std::uint32_t first_idx = shapes_.road_shape_first_idx[road_idx];
  const std::uint32_t count = shapes_.road_shape_count[road_idx];

  std::optional<ShapeGroup> g1;
  std::optional<ShapeGroup> g2;
  FindShapeGroups(shapes_, first_idx, count, s, g1, g2);

  if (!g1.has_value() && !g2.has_value()) {
    return 0.0;
  }
  if (g1.has_value() && !g2.has_value()) {
    return EvaluateGroupTGradient(shapes_, *g1, t);
  }
  if (!g1.has_value() && g2.has_value()) {
    return EvaluateGroupTGradient(shapes_, *g2, t);
  }

  const double g1_val = EvaluateGroupTGradient(shapes_, *g1, t);
  if (g1->s == g2->s) {
    return g1_val;
  }
  const double g2_val = EvaluateGroupTGradient(shapes_, *g2, t);
  const double f = (s - g1->s) / (g2->s - g1->s);
  return ((1.0 - f) * g1_val) + (f * g2_val);
}

}  // namespace strada::cpm
