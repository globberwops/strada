#include <cmath>
#include <limits>
#include <optional>
#include <strada/ast/abstract_syntax_tree.hpp>
#include <strada/cpm/elevation_profile.hpp>
#include <strada/cpm/ids.hpp>
#include <vector>

namespace strada::cpm {

namespace {

struct ShapeGroup {
  double s{};
  std::uint32_t first_idx{};
  std::uint32_t count{};
};

auto BuildShapeGroup(const ShapesSoA& shapes, std::uint32_t first_idx, std::uint32_t count, double target_s) noexcept
    -> ShapeGroup {
  ShapeGroup group{.s = target_s, .first_idx = 0, .count = 0};
  bool in_group = false;
  for (std::uint32_t i = 0; i < count; ++i) {
    const std::uint32_t idx = first_idx + i;
    if (shapes.s[idx] == target_s) {
      if (!in_group) {
        group.first_idx = idx;
        in_group = true;
      }
      group.count++;
    }
  }
  return group;
}

void FindShapeGroups(const ShapesSoA& shapes, std::uint32_t first_idx, std::uint32_t count, double s_coord,
                     std::optional<ShapeGroup>& group1, std::optional<ShapeGroup>& group2) noexcept {
  group1 = std::nullopt;
  group2 = std::nullopt;
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
    group1 = BuildShapeGroup(shapes, first_idx, count, max_s_le);
  }

  if (found_ge) {
    group2 = BuildShapeGroup(shapes, first_idx, count, min_s_ge);
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
  const double delta_t = t_coord - shapes.t[active_idx];
  return shapes.a[active_idx] +
         (delta_t * (shapes.b[active_idx] + delta_t * (shapes.c[active_idx] + delta_t * shapes.d[active_idx])));
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
  constexpr double k_cubic_deriv_factor = 3.0;
  const double delta_t = t_coord - shapes.t[active_idx];
  return shapes.b[active_idx] +
         (delta_t * (2.0 * shapes.c[active_idx] + delta_t * k_cubic_deriv_factor * shapes.d[active_idx]));
}

}  // namespace

ElevationProfile::ElevationProfile(const ast::AbstractSyntaxTree& map) {
  for (const auto& road : map.roads) {
    // Elevation profile compilation
    {
      std::vector<ast::Coefficient> coeffs;
      coeffs.reserve(road.elevation_profile.elevations.size());
      for (const auto& elev : road.elevation_profile.elevations) {
        coeffs.push_back({elev.s, elev.a, elev.b, elev.c, elev.d});
      }
      auto [first_idx, count] = polynomials_.Compile(coeffs);
      elevation_.road_elevation_first_idx.push_back(first_idx);
      elevation_.road_elevation_count.push_back(count);
    }

    // Superelevation profile compilation
    {
      std::vector<ast::Coefficient> coeffs;
      coeffs.reserve(road.lateral_profile.superelevations.size());
      for (const auto& super : road.lateral_profile.superelevations) {
        coeffs.push_back({super.s, super.a, super.b, super.c, super.d});
      }
      auto [first_idx, count] = polynomials_.Compile(coeffs);
      elevation_.road_superelevation_first_idx.push_back(first_idx);
      elevation_.road_superelevation_count.push_back(count);
    }

    // Shape profile compilation
    {
      auto first_idx = static_cast<std::uint32_t>(shapes_.s.size());
      auto count = static_cast<std::uint32_t>(road.lateral_profile.shapes.size());
      for (const auto& shape : road.lateral_profile.shapes) {
        shapes_.s.push_back(shape.s);
        shapes_.t.push_back(shape.t);
        shapes_.a.push_back(shape.a);
        shapes_.b.push_back(shape.b);
        shapes_.c.push_back(shape.c);
        shapes_.d.push_back(shape.d);
      }
      shapes_.road_shape_first_idx.push_back(first_idx);
      shapes_.road_shape_count.push_back(count);
    }

    // Record whether road has cross section surface strips
    bool has_css = road.lateral_profile.cross_section_surface.has_value();
    road_has_css_.push_back(static_cast<std::uint8_t>(has_css));
  }
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

  elev = polynomials_.Evaluate(elevation_.road_elevation_first_idx[road_idx], elevation_.road_elevation_count[road_idx],
                               s);

  double d_elev = polynomials_.EvaluateDerivative(elevation_.road_elevation_first_idx[road_idx],
                                                  elevation_.road_elevation_count[road_idx], s);
  natural_pitch = std::atan(d_elev);

  if (!has_css) {
    natural_roll = polynomials_.Evaluate(elevation_.road_superelevation_first_idx[road_idx],
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

  std::optional<ShapeGroup> group1;
  std::optional<ShapeGroup> group2;
  FindShapeGroups(shapes_, first_idx, count, s, group1, group2);

  if (!group1.has_value() && !group2.has_value()) {
    return 0.0;
  }
  if (group1.has_value() && !group2.has_value()) {
    return EvaluateGroupHeight(shapes_, *group1, t);
  }
  if (!group1.has_value() && group2.has_value()) {
    return EvaluateGroupHeight(shapes_, *group2, t);
  }

  const double height1 = EvaluateGroupHeight(shapes_, *group1, t);
  if (group1->s == group2->s) {
    return height1;
  }
  const double height2 = EvaluateGroupHeight(shapes_, *group2, t);
  const double f = (s - group1->s) / (group2->s - group1->s);
  return ((1.0 - f) * height1) + (f * height2);
}

auto ElevationProfile::EvaluateShapeTGradient(RoadId road, double s, double t) const noexcept -> double {
  auto road_idx = static_cast<std::uint32_t>(road);
  if (road_idx >= shapes_.road_shape_first_idx.size()) {
    return 0.0;
  }

  const std::uint32_t first_idx = shapes_.road_shape_first_idx[road_idx];
  const std::uint32_t count = shapes_.road_shape_count[road_idx];

  std::optional<ShapeGroup> group1;
  std::optional<ShapeGroup> group2;
  FindShapeGroups(shapes_, first_idx, count, s, group1, group2);

  if (!group1.has_value() && !group2.has_value()) {
    return 0.0;
  }
  if (group1.has_value() && !group2.has_value()) {
    return EvaluateGroupTGradient(shapes_, *group1, t);
  }
  if (!group1.has_value() && group2.has_value()) {
    return EvaluateGroupTGradient(shapes_, *group2, t);
  }

  const double grad1 = EvaluateGroupTGradient(shapes_, *group1, t);
  if (group1->s == group2->s) {
    return grad1;
  }
  const double grad2 = EvaluateGroupTGradient(shapes_, *group2, t);
  const double factor = (s - group1->s) / (group2->s - group1->s);
  return ((1.0 - factor) * grad1) + (factor * grad2);
}

}  // namespace strada::cpm
