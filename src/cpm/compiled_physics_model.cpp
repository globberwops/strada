#include <algorithm>
#include <cmath>
#include <limits>
#include <strada/cpm/compiled_physics_model.hpp>

namespace strada::cpm {

namespace {

auto EvaluatePolynomial(const PolynomialsSoA& poly, uint32_t first_idx, uint32_t count, double s) noexcept -> double {
  if (count == 0) {
    return 0.0;
  }
  uint32_t active_idx = first_idx;
  for (uint32_t i = 0; i < count; ++i) {
    uint32_t idx = first_idx + i;
    if (s >= poly.s_start[idx]) {
      active_idx = idx;
    } else {
      break;
    }
  }
  double ds = s - poly.s_start[active_idx];
  return poly.a[active_idx] + poly.b[active_idx] * ds + poly.c[active_idx] * ds * ds +
         poly.d[active_idx] * ds * ds * ds;
}

void CompileCoefficients(const std::vector<ast::Coefficient>& coeffs, PolynomialsSoA& dest, uint32_t& first_idx,
                         uint32_t& count) {
  first_idx = static_cast<uint32_t>(dest.s_start.size());
  count = static_cast<uint32_t>(coeffs.size());
  for (const auto& c : coeffs) {
    dest.s_start.push_back(c.s);
    dest.a.push_back(c.a);
    dest.b.push_back(c.b);
    dest.c.push_back(c.c);
    dest.d.push_back(c.d);
  }
}

auto EvaluateStripOwnHeight(const PolynomialsSoA& poly, const StripsSoA& strips, uint32_t strip_idx, double s,
                            double dt) noexcept -> double {
  double c0 = EvaluatePolynomial(poly, strips.c0_first_idx[strip_idx], strips.c0_count[strip_idx], s);
  double c1 = EvaluatePolynomial(poly, strips.c1_first_idx[strip_idx], strips.c1_count[strip_idx], s);
  double c2 = EvaluatePolynomial(poly, strips.c2_first_idx[strip_idx], strips.c2_count[strip_idx], s);
  double c3 = EvaluatePolynomial(poly, strips.c3_first_idx[strip_idx], strips.c3_count[strip_idx], s);
  return c0 + c1 * dt + c2 * dt * dt + c3 * dt * dt * dt;
}

auto EvaluateStripHeight(const PolynomialsSoA& poly, const StripsSoA& strips, uint32_t strip_idx,
                         uint32_t first_strip_idx, uint32_t strip_count, double s, double dt) noexcept -> double {
  double h_accum = EvaluateStripOwnHeight(poly, strips, strip_idx, s, dt);
  uint32_t curr_strip_idx = strip_idx;

  while (strips.is_relative[curr_strip_idx]) {
    int32_t id = strips.strip_id[curr_strip_idx];
    int32_t inner_id = (id > 0) ? (id - 1) : (id + 1);

    bool found = false;
    for (uint32_t j = 0; j < strip_count; ++j) {
      uint32_t inner_idx = first_strip_idx + j;
      if (strips.strip_id[inner_idx] == inner_id) {
        double inner_w = EvaluatePolynomial(poly, strips.width_first_idx[inner_idx], strips.width_count[inner_idx], s);
        double inner_dt = (inner_id > 0) ? inner_w : -inner_w;
        h_accum += EvaluateStripOwnHeight(poly, strips, inner_idx, s, inner_dt);
        curr_strip_idx = inner_idx;
        found = true;
        break;
      }
    }
    if (!found) {
      break;
    }
  }
  return h_accum;
}

}  // namespace

auto CompiledPhysicsModel::RoadToInertial(RoadPose pose, QueryContext&) const noexcept -> InertialPose {
  double h_surf = 0.0;
  uint32_t road_idx = static_cast<uint32_t>(pose.road);

  if (road_idx < road_css_.strip_count.size()) {
    uint32_t strip_count = road_css_.strip_count[road_idx];
    if (strip_count > 0) {
      // Evaluate tOffset(s)
      double t_offset = EvaluatePolynomial(polynomials_, road_css_.t_offset_first_idx[road_idx],
                                           road_css_.t_offset_count[road_idx], pose.s);

      double t_surf = pose.t - t_offset;
      bool is_left = (t_surf >= 0.0);
      double t_target = is_left ? t_surf : std::abs(t_surf);

      uint32_t first_strip_idx = road_css_.first_strip_idx[road_idx];
      double t_accum = 0.0;

      for (uint32_t i = 0; i < strip_count; ++i) {
        uint32_t strip_idx = first_strip_idx + i;
        int32_t id = strips_.strip_id[strip_idx];
        bool strip_is_left = (id > 0);

        if (strip_is_left == is_left) {
          // Evaluate width
          double w = std::numeric_limits<double>::infinity();
          uint32_t w_count = strips_.width_count[strip_idx];
          if (w_count > 0) {
            w = EvaluatePolynomial(polynomials_, strips_.width_first_idx[strip_idx], w_count, pose.s);
          }

          if (t_target >= t_accum && t_target < t_accum + w) {
            // Found strip
            double t_strip = t_target - t_accum;
            double dt = strip_is_left ? t_strip : -t_strip;

            h_surf = EvaluateStripHeight(polynomials_, strips_, strip_idx, first_strip_idx, strip_count, pose.s, dt);
            break;
          }
          t_accum += w;
        }
      }
    }
  }

  // Simplified natural road coordinates for tracer bullet: straight along X-axis
  InertialPose ip;
  ip.x = pose.s;
  ip.y = pose.t;
  ip.z = h_surf + pose.h;
  ip.heading = pose.heading;
  ip.pitch = pose.pitch;
  ip.roll = pose.roll;
  return ip;
}

auto CompiledPhysicsModel::road_count() const noexcept -> std::size_t { return road_string_ids_.size(); }

auto CompiledPhysicsModel::road_id_from_string(std::string_view original_id) const noexcept -> std::optional<RoadId> {
  auto it = std::find(road_string_ids_.begin(), road_string_ids_.end(), original_id);
  if (it != road_string_ids_.end()) {
    return static_cast<RoadId>(std::distance(road_string_ids_.begin(), it));
  }
  return std::nullopt;
}

auto CompiledPhysicsModel::original_road_id(RoadId id) const noexcept -> std::string_view {
  uint32_t idx = static_cast<uint32_t>(id);
  if (idx < road_string_ids_.size()) {
    return road_string_ids_[idx];
  }
  return "";
}

auto BuildCompiledPhysicsModel(const ast::AbstractSyntaxTree& map) -> CompiledPhysicsModel {
  CompiledPhysicsModel model;

  for (const auto& road : map.roads) {
    model.road_string_ids_.push_back(road.id);

    if (road.lateral_profile.cross_section_surface.has_value()) {
      const auto& css = *road.lateral_profile.cross_section_surface;

      model.road_css_.first_strip_idx.push_back(static_cast<uint32_t>(model.strips_.strip_id.size()));
      model.road_css_.strip_count.push_back(static_cast<uint32_t>(css.strips.size()));

      uint32_t t_off_idx = 0;
      uint32_t t_off_cnt = 0;
      CompileCoefficients(css.t_offset, model.polynomials_, t_off_idx, t_off_cnt);
      model.road_css_.t_offset_first_idx.push_back(t_off_idx);
      model.road_css_.t_offset_count.push_back(t_off_cnt);

      // Sort strips by ascending absolute ID value to guarantee correct inside-out ordering
      auto sorted_strips = css.strips;
      std::sort(sorted_strips.begin(), sorted_strips.end(),
                [](const auto& a, const auto& b) { return std::abs(a.id) < std::abs(b.id); });

      for (const auto& strip : sorted_strips) {
        model.strips_.strip_id.push_back(strip.id);
        model.strips_.is_relative.push_back(strip.mode == "relative");

        uint32_t first_idx = 0, count = 0;

        CompileCoefficients(strip.width, model.polynomials_, first_idx, count);
        model.strips_.width_first_idx.push_back(first_idx);
        model.strips_.width_count.push_back(count);

        CompileCoefficients(strip.constant, model.polynomials_, first_idx, count);
        model.strips_.c0_first_idx.push_back(first_idx);
        model.strips_.c0_count.push_back(count);

        CompileCoefficients(strip.linear, model.polynomials_, first_idx, count);
        model.strips_.c1_first_idx.push_back(first_idx);
        model.strips_.c1_count.push_back(count);

        CompileCoefficients(strip.quadratic, model.polynomials_, first_idx, count);
        model.strips_.c2_first_idx.push_back(first_idx);
        model.strips_.c2_count.push_back(count);

        CompileCoefficients(strip.cubic, model.polynomials_, first_idx, count);
        model.strips_.c3_first_idx.push_back(first_idx);
        model.strips_.c3_count.push_back(count);
      }
    } else {
      model.road_css_.first_strip_idx.push_back(0);
      model.road_css_.strip_count.push_back(0);
      model.road_css_.t_offset_first_idx.push_back(0);
      model.road_css_.t_offset_count.push_back(0);
    }
  }

  return model;
}

}  // namespace strada::cpm
