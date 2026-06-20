#include <algorithm>
#include <cmath>
#include <limits>
#include <strada/cpm/aligned_allocator.hpp>
#include <strada/cpm/compiled_physics_model.hpp>

#include "geometry_math.hpp"
#include "rotation.hpp"

namespace strada::cpm {

namespace {

constexpr double kCurvatureThreshold = 1e-12;
constexpr double kPolyCoeff2 = 2.0;
constexpr double kPolyCoeff3 = 3.0;

auto EvaluatePolynomial(const PolynomialsSoA& poly, uint32_t first_idx, uint32_t count, double s_coord) noexcept
    -> double {
  if (count == 0) {
    return 0.0;
  }
  uint32_t active_idx = first_idx;
  for (uint32_t i = 0; i < count; ++i) {
    uint32_t idx = first_idx + i;
    if (s_coord >= poly.s_start[idx]) {
      active_idx = idx;
    } else {
      break;
    }
  }
  double ds_val = s_coord - poly.s_start[active_idx];
  return poly.a[active_idx] + (poly.b[active_idx] * ds_val) + (poly.c[active_idx] * ds_val * ds_val) +
         (poly.d[active_idx] * ds_val * ds_val * ds_val);
}

void CompileCoefficients(const std::vector<ast::Coefficient>& coeffs, PolynomialsSoA& dest, uint32_t& first_idx,
                         uint32_t& count) {
  first_idx = static_cast<uint32_t>(dest.s_start.size());
  count = static_cast<uint32_t>(coeffs.size());
  for (const auto& coeff : coeffs) {
    dest.s_start.push_back(coeff.s);
    dest.a.push_back(coeff.a);
    dest.b.push_back(coeff.b);
    dest.c.push_back(coeff.c);
    dest.d.push_back(coeff.d);
  }
}

auto EvaluateStripOwnHeight(const PolynomialsSoA& poly, const StripsSoA& strips, uint32_t strip_idx, double s_coord,
                            double dt_val) noexcept -> double {
  double coeff0 = EvaluatePolynomial(poly, strips.c0_first_idx[strip_idx], strips.c0_count[strip_idx], s_coord);
  double coeff1 = EvaluatePolynomial(poly, strips.c1_first_idx[strip_idx], strips.c1_count[strip_idx], s_coord);
  double coeff2 = EvaluatePolynomial(poly, strips.c2_first_idx[strip_idx], strips.c2_count[strip_idx], s_coord);
  double coeff3 = EvaluatePolynomial(poly, strips.c3_first_idx[strip_idx], strips.c3_count[strip_idx], s_coord);
  return coeff0 + (coeff1 * dt_val) + (coeff2 * dt_val * dt_val) + (coeff3 * dt_val * dt_val * dt_val);
}

auto EvaluateStripHeight(const PolynomialsSoA& poly, const StripsSoA& strips, uint32_t strip_idx,
                         uint32_t first_strip_idx, uint32_t strip_count, double s_coord, double dt_val) noexcept
    -> double {
  double h_accum = EvaluateStripOwnHeight(poly, strips, strip_idx, s_coord, dt_val);
  uint32_t curr_strip_idx = strip_idx;

  while (strips.is_relative[curr_strip_idx] != 0U) {
    int32_t id_val = strips.strip_id[curr_strip_idx];
    int32_t inner_id = (id_val > 0) ? (id_val - 1) : (id_val + 1);

    bool found = false;
    for (uint32_t j = 0; j < strip_count; ++j) {
      uint32_t inner_idx = first_strip_idx + j;
      if (strips.strip_id[inner_idx] == inner_id) {
        double inner_w =
            EvaluatePolynomial(poly, strips.width_first_idx[inner_idx], strips.width_count[inner_idx], s_coord);
        double inner_dt = (inner_id > 0) ? inner_w : -inner_w;
        h_accum += EvaluateStripOwnHeight(poly, strips, inner_idx, s_coord, inner_dt);
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

auto FindSegmentIndex(const ReferenceLineSoA& ref_line, RoadPose pose, QueryContext& ctx, uint32_t first_seg,
                      uint32_t seg_count) noexcept -> uint32_t {
  uint32_t seg_idx = first_seg;
  bool hit = false;

  if (ctx.last_road == pose.road && ctx.last_segment_idx.has_value()) {
    uint32_t last_idx = *ctx.last_segment_idx;
    if (last_idx >= first_seg && last_idx < first_seg + seg_count) {
      double s_start = ref_line.s_offset[last_idx];
      double s_end = s_start + ref_line.length[last_idx];
      if (pose.s >= s_start && pose.s < s_end) [[likely]] {
        seg_idx = last_idx;
        hit = true;
      }
    }
  }

  if (!hit) [[unlikely]] {
    for (uint32_t i = 0; i < seg_count; ++i) {
      uint32_t idx = first_seg + i;
      if (pose.s >= ref_line.s_offset[idx]) {
        seg_idx = idx;
      } else {
        break;
      }
    }
    ctx.last_road = pose.road;
    ctx.last_segment_idx = seg_idx;
  }
  return seg_idx;
}

void EvaluateReferenceLine(const ReferenceLineSoA& ref_line, const AlignedVector<double>& arc_curvature,
                           uint32_t seg_idx, double s_coord, double& ref_x, double& ref_y,
                           double& tangent_hdg) noexcept {
  double ds_val = s_coord - ref_line.s_offset[seg_idx];
  ref_x = ref_line.x[seg_idx];
  ref_y = ref_line.y[seg_idx];
  double ref_hdg = ref_line.hdg[seg_idx];
  tangent_hdg = ref_hdg;

  GeometryType type = ref_line.type[seg_idx];
  uint32_t type_idx = ref_line.type_index[seg_idx];

  if (type == GeometryType::kLine) {
    ref_x += ds_val * std::cos(ref_hdg);
    ref_y += ds_val * std::sin(ref_hdg);
  } else if (type == GeometryType::kArc) {
    double curvature = arc_curvature[type_idx];
    if (std::abs(curvature) > kCurvatureThreshold) {
      tangent_hdg += curvature * ds_val;
      ref_x += (1.0 / curvature) * (std::sin(tangent_hdg) - std::sin(ref_hdg));
      ref_y -= (1.0 / curvature) * (std::cos(tangent_hdg) - std::cos(ref_hdg));
    } else {
      ref_x += ds_val * std::cos(ref_hdg);
      ref_y += ds_val * std::sin(ref_hdg);
    }
  } else if (type == GeometryType::kSpiral) {
    double curv_start = ref_line.spiral_curv_start[seg_idx];
    double curv_end = ref_line.spiral_curv_end[seg_idx];
    double length = ref_line.length[seg_idx];
    double lambda = (length > 0.0) ? ((curv_end - curv_start) / length) : 0.0;

    tangent_hdg += (curv_start * ds_val) + (0.5 * (lambda * (ds_val * ds_val)));

    double param_a = lambda * ds_val * ds_val;
    double param_b = curv_start * ds_val;
    auto [local_x, local_y] = EvaluateClothoidIntegrals(param_a, param_b);

    ref_x += ds_val * (std::cos(ref_hdg) * local_x - std::sin(ref_hdg) * local_y);
    ref_y += ds_val * (std::sin(ref_hdg) * local_x + std::cos(ref_hdg) * local_y);
  } else if (type == GeometryType::kParamPoly3) {
    double a_u = ref_line.pp3_a_u[seg_idx];
    double b_u = ref_line.pp3_b_u[seg_idx];
    double c_u = ref_line.pp3_c_u[seg_idx];
    double d_u = ref_line.pp3_d_u[seg_idx];
    double a_v = ref_line.pp3_a_v[seg_idx];
    double b_v = ref_line.pp3_b_v[seg_idx];
    double c_v = ref_line.pp3_c_v[seg_idx];
    double d_v = ref_line.pp3_d_v[seg_idx];
    uint8_t p_range = ref_line.pp3_p_range[seg_idx];

    double length = ref_line.length[seg_idx];
    double p_val = ds_val;
    if (p_range == 0U) {
      p_val = (length > 0.0) ? (ds_val / length) : 0.0;
    }

    double u_p = a_u + (p_val * (b_u + (p_val * (c_u + (d_u * p_val)))));
    double v_p = a_v + (p_val * (b_v + (p_val * (c_v + (d_v * p_val)))));

    double du_dp = b_u + (p_val * ((kPolyCoeff2 * c_u) + (kPolyCoeff3 * d_u * p_val)));
    double dv_dp = b_v + (p_val * ((kPolyCoeff2 * c_v) + (kPolyCoeff3 * d_v * p_val)));

    double cos_hdg = std::cos(ref_hdg);
    double sin_hdg = std::sin(ref_hdg);

    ref_x += (u_p * cos_hdg) - (v_p * sin_hdg);
    ref_y += (u_p * sin_hdg) + (v_p * cos_hdg);
    tangent_hdg += std::atan2(dv_dp, du_dp);
  }
}

void EvaluateNaturalOrientationAndElev(const PolynomialsSoA& polynomials,
                                       const std::vector<uint32_t>& road_elevation_first_idx,
                                       const std::vector<uint32_t>& road_elevation_count,
                                       const std::vector<uint32_t>& road_superelevation_first_idx,
                                       const std::vector<uint32_t>& road_superelevation_count,
                                       const RoadCrossSectionSurfaceSoA& road_css, uint32_t road_idx, double s_coord,
                                       double& elev, double& natural_pitch, double& natural_roll) noexcept {
  elev = EvaluatePolynomial(polynomials, road_elevation_first_idx[road_idx], road_elevation_count[road_idx], s_coord);

  double d_elev = 0.0;
  uint32_t elev_first_idx = road_elevation_first_idx[road_idx];
  uint32_t elev_count = road_elevation_count[road_idx];
  if (elev_count > 0) {
    uint32_t active_idx = elev_first_idx;
    for (uint32_t i = 0; i < elev_count; ++i) {
      uint32_t idx = elev_first_idx + i;
      if (s_coord >= polynomials.s_start[idx]) {
        active_idx = idx;
      } else {
        break;
      }
    }
    double ds_poly = s_coord - polynomials.s_start[active_idx];
    d_elev = polynomials.b[active_idx] + (kPolyCoeff2 * polynomials.c[active_idx] * ds_poly) +
             (kPolyCoeff3 * polynomials.d[active_idx] * ds_poly * ds_poly);
  }
  natural_pitch = std::atan(d_elev);

  natural_roll = 0.0;
  if (!road_css.strip_count.empty() && road_css.strip_count[road_idx] == 0) {
    natural_roll = EvaluatePolynomial(polynomials, road_superelevation_first_idx[road_idx],
                                      road_superelevation_count[road_idx], s_coord);
  }
}

void EvaluateCrossSectionSurfaceOffset(const PolynomialsSoA& polynomials, const StripsSoA& strips,
                                       const RoadCrossSectionSurfaceSoA& road_css, uint32_t road_idx, double s_coord,
                                       double t_coord, double& h_surf) noexcept {
  h_surf = 0.0;
  uint32_t css_strip_count = road_css.strip_count.empty() ? 0 : road_css.strip_count[road_idx];
  if (css_strip_count > 0) {
    double t_offset = EvaluatePolynomial(polynomials, road_css.t_offset_first_idx[road_idx],
                                         road_css.t_offset_count[road_idx], s_coord);

    double t_surf = t_coord - t_offset;
    bool is_left = (t_surf >= 0.0);
    double t_target = is_left ? t_surf : std::abs(t_surf);

    uint32_t first_strip_idx = road_css.first_strip_idx[road_idx];
    double t_accum = 0.0;

    for (uint32_t i = 0; i < css_strip_count; ++i) {
      uint32_t strip_idx = first_strip_idx + i;
      int32_t id_val = strips.strip_id[strip_idx];
      bool strip_is_left = (id_val > 0);

      if (strip_is_left == is_left) {
        double width_val = std::numeric_limits<double>::infinity();
        uint32_t w_count = strips.width_count[strip_idx];
        if (w_count > 0) {
          width_val = EvaluatePolynomial(polynomials, strips.width_first_idx[strip_idx], w_count, s_coord);
        }

        if (t_target >= t_accum && t_target < t_accum + width_val) {
          double t_strip = t_target - t_accum;
          double dt_val = strip_is_left ? t_strip : -t_strip;

          h_surf =
              EvaluateStripHeight(polynomials, strips, strip_idx, first_strip_idx, css_strip_count, s_coord, dt_val);
          break;
        }
        t_accum += width_val;
      }
    }
  }
}

struct Aabb {
  double min_x{std::numeric_limits<double>::max()};
  double min_y{std::numeric_limits<double>::max()};
  double max_x{-std::numeric_limits<double>::max()};
  double max_y{-std::numeric_limits<double>::max()};

  void Grow(const Aabb& other) noexcept {
    min_x = std::min(min_x, other.min_x);
    min_y = std::min(min_y, other.min_y);
    max_x = std::max(max_x, other.max_x);
    max_y = std::max(max_y, other.max_y);
  }

  void Grow(double px, double py) noexcept {
    min_x = std::min(min_x, px);
    min_y = std::min(min_y, py);
    max_x = std::max(max_x, px);
    max_y = std::max(max_y, py);
  }

  [[nodiscard]] auto Area() const noexcept -> double {
    double dx = max_x - min_x;
    double dy = max_y - min_y;
    return (dx > 0.0 ? dx : 0.0) + (dy > 0.0 ? dy : 0.0);
  }
};

auto MakeLeafNode(std::vector<BvhNode>& nodes, uint32_t node_idx, const Aabb& bounds,
                  std::vector<BvhPrimitiveInfo>& final_primitives, const std::vector<uint32_t>& prim_indices,
                  const std::vector<BvhPrimitiveInfo>& temp_primitives, uint32_t start_idx, uint32_t count) noexcept
    -> uint32_t {
  auto prim_start = static_cast<uint32_t>(final_primitives.size());
  for (uint32_t idx = 0; idx < count; ++idx) {
    final_primitives.push_back(temp_primitives[prim_indices[start_idx + idx]]);
  }

  nodes[node_idx].min_x = bounds.min_x;
  nodes[node_idx].min_y = bounds.min_y;
  nodes[node_idx].max_x = bounds.max_x;
  nodes[node_idx].max_y = bounds.max_y;
  nodes[node_idx].left = prim_start;
  nodes[node_idx].right = count | 0x80000000;

  return node_idx;
}

auto BuildBvhRecursive(std::vector<BvhNode>& nodes, std::vector<BvhPrimitiveInfo>& final_primitives,
                       std::vector<uint32_t>& prim_indices, const std::vector<BvhPrimitiveInfo>& temp_primitives,
                       const std::vector<Aabb>& temp_aabbs, uint32_t start_idx, uint32_t end_idx) noexcept -> uint32_t {
  auto node_idx = static_cast<uint32_t>(nodes.size());
  nodes.push_back(BvhNode{});

  Aabb bounds;
  Aabb centroid_bounds;
  for (uint32_t idx = start_idx; idx < end_idx; ++idx) {
    uint32_t prim_idx = prim_indices[idx];
    bounds.Grow(temp_aabbs[prim_idx]);
    double cx = 0.5 * (temp_aabbs[prim_idx].min_x + temp_aabbs[prim_idx].max_x);
    double cy = 0.5 * (temp_aabbs[prim_idx].min_y + temp_aabbs[prim_idx].max_y);
    centroid_bounds.Grow(cx, cy);
  }

  uint32_t count = end_idx - start_idx;
  constexpr uint32_t kLeafThreshold = 4;

  if (count <= kLeafThreshold) {
    return MakeLeafNode(nodes, node_idx, bounds, final_primitives, prim_indices, temp_primitives, start_idx, count);
  }

  double ext_x = centroid_bounds.max_x - centroid_bounds.min_x;
  double ext_y = centroid_bounds.max_y - centroid_bounds.min_y;
  int axis = (ext_x > ext_y) ? 0 : 1;

  double min_coord = (axis == 0) ? centroid_bounds.min_x : centroid_bounds.min_y;
  double max_coord = (axis == 0) ? centroid_bounds.max_x : centroid_bounds.max_y;

  if (max_coord - min_coord < 1e-9) {
    return MakeLeafNode(nodes, node_idx, bounds, final_primitives, prim_indices, temp_primitives, start_idx, count);
  }

  constexpr int kNumBins = 16;
  struct Bin {
    uint32_t count{0};
    Aabb bounds;
  };
  std::array<Bin, kNumBins> bins{};

  double scale = kNumBins / (max_coord - min_coord);
  for (uint32_t idx = start_idx; idx < end_idx; ++idx) {
    uint32_t prim_idx = prim_indices[idx];
    double centroid = (axis == 0) ? (0.5 * (temp_aabbs[prim_idx].min_x + temp_aabbs[prim_idx].max_x))
                                  : (0.5 * (temp_aabbs[prim_idx].min_y + temp_aabbs[prim_idx].max_y));
    int bin_idx = static_cast<int>((centroid - min_coord) * scale);
    bin_idx = std::clamp(bin_idx, 0, kNumBins - 1);
    bins[bin_idx].count++;
    bins[bin_idx].bounds.Grow(temp_aabbs[prim_idx]);
  }

  double min_split_cost = std::numeric_limits<double>::max();
  int best_split_bin = -1;

  std::array<Aabb, kNumBins - 1> left_bounds{};
  std::array<uint32_t, kNumBins - 1> left_counts{};
  Aabb left_accum;
  uint32_t left_cnt = 0;
  for (int idx = 0; idx < kNumBins - 1; ++idx) {
    left_accum.Grow(bins[idx].bounds);
    left_cnt += bins[idx].count;
    left_bounds[idx] = left_accum;
    left_counts[idx] = left_cnt;
  }

  std::array<Aabb, kNumBins - 1> right_bounds{};
  std::array<uint32_t, kNumBins - 1> right_counts{};
  Aabb right_accum;
  uint32_t right_cnt = 0;
  for (int idx = kNumBins - 1; idx > 0; --idx) {
    right_accum.Grow(bins[idx].bounds);
    right_cnt += bins[idx].count;
    right_bounds[idx - 1] = right_accum;
    right_counts[idx - 1] = right_cnt;
  }

  double parent_area = bounds.Area();
  constexpr double kCTrav = 1.0;
  constexpr double kCIsect = 1.0;

  for (int idx = 0; idx < kNumBins - 1; ++idx) {
    if (left_counts[idx] == 0 || right_counts[idx] == 0) {
      continue;
    }
    double cost =
        kCTrav +
        (kCIsect * (left_bounds[idx].Area() * left_counts[idx] + right_bounds[idx].Area() * right_counts[idx]) /
         parent_area);
    if (cost < min_split_cost) {
      min_split_cost = cost;
      best_split_bin = idx;
    }
  }

  double no_split_cost = count * kCIsect;

  if (min_split_cost >= no_split_cost) {
    return MakeLeafNode(nodes, node_idx, bounds, final_primitives, prim_indices, temp_primitives, start_idx, count);
  }

  auto split_it = std::stable_partition(
      prim_indices.begin() + start_idx, prim_indices.begin() + end_idx, [&](uint32_t prim_idx) -> bool {
        double centroid = (axis == 0) ? (0.5 * (temp_aabbs[prim_idx].min_x + temp_aabbs[prim_idx].max_x))
                                      : (0.5 * (temp_aabbs[prim_idx].min_y + temp_aabbs[prim_idx].max_y));
        int bin_idx = static_cast<int>((centroid - min_coord) * scale);
        bin_idx = std::clamp(bin_idx, 0, kNumBins - 1);
        return bin_idx <= best_split_bin;
      });

  auto mid_idx = static_cast<uint32_t>(std::distance(prim_indices.begin(), split_it));

  if (mid_idx == start_idx || mid_idx == end_idx) {
    mid_idx = start_idx + (count / 2);
  }

  uint32_t left_child =
      BuildBvhRecursive(nodes, final_primitives, prim_indices, temp_primitives, temp_aabbs, start_idx, mid_idx);
  uint32_t right_child =
      BuildBvhRecursive(nodes, final_primitives, prim_indices, temp_primitives, temp_aabbs, mid_idx, end_idx);

  nodes[node_idx].min_x = bounds.min_x;
  nodes[node_idx].min_y = bounds.min_y;
  nodes[node_idx].max_x = bounds.max_x;
  nodes[node_idx].max_y = bounds.max_y;
  nodes[node_idx].left = left_child;
  nodes[node_idx].right = right_child;

  return node_idx;
}

auto ComputeSegmentAabb(const ReferenceLineSoA& ref_line, const AlignedVector<double>& arc_curvature, uint32_t seg_idx,
                        double inflation_radius) noexcept -> Aabb {
  double min_x = std::numeric_limits<double>::max();
  double min_y = std::numeric_limits<double>::max();
  double max_x = -std::numeric_limits<double>::max();
  double max_y = -std::numeric_limits<double>::max();

  double length = ref_line.length[seg_idx];
  double s_start = ref_line.s_offset[seg_idx];

  int num_samples = 1;
  if (ref_line.type[seg_idx] != GeometryType::kLine) {
    num_samples = 32;
  }

  for (int idx = 0; idx <= num_samples; ++idx) {
    double s_local = (static_cast<double>(idx) / num_samples) * length;
    double rx = 0.0;
    double ry = 0.0;
    double r_hdg = 0.0;
    EvaluateReferenceLine(ref_line, arc_curvature, seg_idx, s_start + s_local, rx, ry, r_hdg);
    min_x = std::min(min_x, rx);
    min_y = std::min(min_y, ry);
    max_x = std::max(max_x, rx);
    max_y = std::max(max_y, ry);
  }

  Aabb bounds;
  bounds.min_x = min_x - inflation_radius;
  bounds.min_y = min_y - inflation_radius;
  bounds.max_x = max_x + inflation_radius;
  bounds.max_y = max_y + inflation_radius;
  return bounds;
}

auto EvaluateAstLaneWidth(const ast::Lane& lane, double s_local_to_section) noexcept -> double {
  if (lane.widths.empty()) {
    return 0.0;
  }
  const ast::LaneWidth* active = lane.widths.data();
  for (const auto& width_poly : lane.widths) {
    if (s_local_to_section >= width_poly.s_offset) {
      active = &width_poly;
    } else {
      break;
    }
  }
  double ds = s_local_to_section - active->s_offset;
  return active->a + (active->b * ds) + (active->c * ds * ds) + (active->d * ds * ds * ds);
}

inline auto TransposeMatrix(const Matrix3x3& mat) noexcept -> Matrix3x3 {
  Matrix3x3 res;
  res[0][0] = mat[0][0];
  res[0][1] = mat[1][0];
  res[0][2] = mat[2][0];
  res[1][0] = mat[0][1];
  res[1][1] = mat[1][1];
  res[1][2] = mat[2][1];
  res[2][0] = mat[0][2];
  res[2][1] = mat[1][2];
  res[2][2] = mat[2][2];
  return res;
}

inline auto DistancePointToAabb(double px, double py, double min_x, double min_y, double max_x, double max_y) noexcept
    -> double {
  double dx = std::max({0.0, min_x - px, px - max_x});
  double dy = std::max({0.0, min_y - py, py - max_y});
  return std::sqrt((dx * dx) + (dy * dy));
}

auto ProjectToGenericSegment(const ReferenceLineSoA& ref_line, const AlignedVector<double>& arc_curvature,
                             uint32_t seg_idx, double seg_length, double px, double py) noexcept -> double {
  constexpr int kNumIntervals = 10;
  double best_s = 0.0;
  double min_dist_sq = std::numeric_limits<double>::max();
  double s_start = ref_line.s_offset[seg_idx];

  for (int i = 0; i <= kNumIntervals; ++i) {
    double s_test = (static_cast<double>(i) / kNumIntervals) * seg_length;
    double rx = 0.0;
    double ry = 0.0;
    double r_hdg = 0.0;
    EvaluateReferenceLine(ref_line, arc_curvature, seg_idx, s_start + s_test, rx, ry, r_hdg);
    double dx = px - rx;
    double dy = py - ry;
    double dist_sq = (dx * dx) + (dy * dy);
    if (dist_sq < min_dist_sq) {
      min_dist_sq = dist_sq;
      best_s = s_test;
    }
  }

  double left_s = std::max(0.0, best_s - (seg_length / kNumIntervals));
  double right_s = std::min(seg_length, best_s + (seg_length / kNumIntervals));
  for (int iter = 0; iter < 30; ++iter) {
    double m1 = left_s + ((right_s - left_s) / 3.0);
    double m2 = right_s - ((right_s - left_s) / 3.0);
    double rx1 = 0.0;
    double ry1 = 0.0;
    double rhdg1 = 0.0;
    double rx2 = 0.0;
    double ry2 = 0.0;
    double rhdg2 = 0.0;
    EvaluateReferenceLine(ref_line, arc_curvature, seg_idx, s_start + m1, rx1, ry1, rhdg1);
    EvaluateReferenceLine(ref_line, arc_curvature, seg_idx, s_start + m2, rx2, ry2, rhdg2);
    double dist1 = ((px - rx1) * (px - rx1)) + ((py - ry1) * (py - ry1));
    double dist2 = ((px - rx2) * (px - rx2)) + ((py - ry2) * (py - ry2));
    if (dist1 < dist2) {
      right_s = m2;
    } else {
      left_s = m1;
    }
  }
  return 0.5 * (left_s + right_s);
}

struct ShapeGroup {
  double s{};
  uint32_t first_idx{};
  uint32_t count{};
};

void FindShapeGroups(const ShapesSoA& shapes, uint32_t first_idx, uint32_t count, double s_coord,
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

  for (uint32_t i = 0; i < count; ++i) {
    double s_val = shapes.s[first_idx + i];
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
    for (uint32_t i = 0; i < count; ++i) {
      uint32_t idx = first_idx + i;
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
    for (uint32_t i = 0; i < count; ++i) {
      uint32_t idx = first_idx + i;
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
  uint32_t active_idx = group.first_idx;
  for (uint32_t i = 0; i < group.count; ++i) {
    uint32_t idx = group.first_idx + i;
    if (t_coord >= shapes.t[idx]) {
      active_idx = idx;
    } else {
      break;
    }
  }
  double dt = t_coord - shapes.t[active_idx];
  return shapes.a[active_idx] + (dt * (shapes.b[active_idx] + dt * (shapes.c[active_idx] + dt * shapes.d[active_idx])));
}

auto EvaluateGroupTGradient(const ShapesSoA& shapes, const ShapeGroup& group, double t_coord) noexcept -> double {
  if (group.count == 0) {
    return 0.0;
  }
  uint32_t active_idx = group.first_idx;
  for (uint32_t i = 0; i < group.count; ++i) {
    uint32_t idx = group.first_idx + i;
    if (t_coord >= shapes.t[idx]) {
      active_idx = idx;
    } else {
      break;
    }
  }
  double dt = t_coord - shapes.t[active_idx];
  return shapes.b[active_idx] + (dt * (2.0 * shapes.c[active_idx] + dt * 3.0 * shapes.d[active_idx]));
}

auto EvaluateShapeHeight(const ShapesSoA& shapes, uint32_t first_idx, uint32_t count, double s_coord,
                         double t_coord) noexcept -> double {
  std::optional<ShapeGroup> g1;
  std::optional<ShapeGroup> g2;
  FindShapeGroups(shapes, first_idx, count, s_coord, g1, g2);

  if (!g1.has_value() && !g2.has_value()) {
    return 0.0;
  }
  if (g1.has_value() && !g2.has_value()) {
    return EvaluateGroupHeight(shapes, *g1, t_coord);
  }
  if (!g1.has_value() && g2.has_value()) {
    return EvaluateGroupHeight(shapes, *g2, t_coord);
  }

  double h1 = EvaluateGroupHeight(shapes, *g1, t_coord);
  if (g1->s == g2->s) {
    return h1;
  }
  double h2 = EvaluateGroupHeight(shapes, *g2, t_coord);
  double f = (s_coord - g1->s) / (g2->s - g1->s);
  return ((1.0 - f) * h1) + (f * h2);
}

auto EvaluateShapeTGradient(const ShapesSoA& shapes, uint32_t first_idx, uint32_t count, double s_coord,
                            double t_coord) noexcept -> double {
  std::optional<ShapeGroup> g1;
  std::optional<ShapeGroup> g2;
  FindShapeGroups(shapes, first_idx, count, s_coord, g1, g2);

  if (!g1.has_value() && !g2.has_value()) {
    return 0.0;
  }
  if (g1.has_value() && !g2.has_value()) {
    return EvaluateGroupTGradient(shapes, *g1, t_coord);
  }
  if (!g1.has_value() && g2.has_value()) {
    return EvaluateGroupTGradient(shapes, *g2, t_coord);
  }

  double g1_val = EvaluateGroupTGradient(shapes, *g1, t_coord);
  if (g1->s == g2->s) {
    return g1_val;
  }
  double g2_val = EvaluateGroupTGradient(shapes, *g2, t_coord);
  double f = (s_coord - g1->s) / (g2->s - g1->s);
  return ((1.0 - f) * g1_val) + (f * g2_val);
}

}  // namespace

[[gnu::hot]] auto CompiledPhysicsModel::RoadToInertial(RoadPose pose, QueryContext& ctx) const noexcept
    -> InertialPose {
  auto road_idx = static_cast<uint32_t>(pose.road);
  if (road_idx >= road_ref_line_count_.size()) {
    return InertialPose{};
  }

  uint32_t first_seg = road_ref_line_first_idx_[road_idx];
  uint32_t seg_count = road_ref_line_count_[road_idx];
  if (seg_count == 0) {
    return InertialPose{};
  }

  // 1. Find segment index
  uint32_t seg_idx = FindSegmentIndex(ref_line_, pose, ctx, first_seg, seg_count);

  // 2. Evaluate reference line
  double ref_x = 0.0;
  double ref_y = 0.0;
  double tangent_hdg = 0.0;
  EvaluateReferenceLine(ref_line_, arc_curvature_, seg_idx, pose.s, ref_x, ref_y, tangent_hdg);

  // 3. Evaluate natural pitch, roll, and elevation
  double elev = 0.0;
  double natural_pitch = 0.0;
  double natural_roll = 0.0;
  EvaluateNaturalOrientationAndElev(polynomials_, road_elevation_first_idx_, road_elevation_count_,
                                    road_superelevation_first_idx_, road_superelevation_count_, road_css_, road_idx,
                                    pose.s, elev, natural_pitch, natural_roll);

  // 4. Cross section surface height offset
  double h_surf = 0.0;
  EvaluateCrossSectionSurfaceOffset(polynomials_, strips_, road_css_, road_idx, pose.s, pose.t, h_surf);

  double h_shape =
      EvaluateShapeHeight(shapes_, road_shape_first_idx_[road_idx], road_shape_count_[road_idx], pose.s, pose.t);
  double shape_grad =
      EvaluateShapeTGradient(shapes_, road_shape_first_idx_[road_idx], road_shape_count_[road_idx], pose.s, pose.t);

  // 5. Position composition
  double roll_total = natural_roll + std::atan(shape_grad);
  Matrix3x3 r_road = EulerToMatrix(tangent_hdg, natural_pitch, roll_total);

  double local_t = pose.t;
  double local_h = pose.h + h_surf + h_shape;

  double offset_x = (r_road[0][1] * local_t) + (r_road[0][2] * local_h);
  double offset_y = (r_road[1][1] * local_t) + (r_road[1][2] * local_h);
  double offset_z = (r_road[2][1] * local_t) + (r_road[2][2] * local_h);

  InertialPose inertial_pose;
  inertial_pose.x = ref_x + offset_x;
  inertial_pose.y = ref_y + offset_y;
  inertial_pose.z = elev + offset_z;

  // Composed orientation composition
  Matrix3x3 r_offset = EulerToMatrix(pose.heading, pose.pitch, pose.roll);
  Matrix3x3 r_inertial = ComposeRotations(r_road, r_offset);

  EulerAngles euler_angles = MatrixToEuler(r_inertial);
  inertial_pose.heading = euler_angles.heading;
  inertial_pose.pitch = euler_angles.pitch;
  inertial_pose.roll = euler_angles.roll;

  return inertial_pose;
}

auto CompiledPhysicsModel::LaneToInertial(LanePose pose, QueryContext& ctx) const noexcept -> InertialPose {
  RoadPose road_pose = LaneToRoad(pose, ctx);
  return RoadToInertial(road_pose, ctx);
}

auto CompiledPhysicsModel::InertialToRoad(InertialPose pose, QueryContext& ctx) const noexcept
    -> std::optional<RoadPose> {
  auto snap_to_road = [&](uint32_t road_idx) noexcept -> std::optional<RoadPose> {
    auto first_seg = road_ref_line_first_idx_[road_idx];
    auto seg_count = road_ref_line_count_[road_idx];
    if (seg_count == 0) {
      return std::nullopt;
    }

    double min_dist_sq = std::numeric_limits<double>::max();
    double best_s = 0.0;
    double best_t = 0.0;
    double best_rhdg = 0.0;

    for (uint32_t i = 0; i < seg_count; ++i) {
      uint32_t seg_idx = first_seg + i;
      double seg_length = ref_line_.length[seg_idx];
      double s_local = 0.0;

      if (ref_line_.type[seg_idx] == GeometryType::kLine) {
        double dx = pose.x - ref_line_.x[seg_idx];
        double dy = pose.y - ref_line_.y[seg_idx];
        double hdg = ref_line_.hdg[seg_idx];
        double ds = (dx * std::cos(hdg)) + (dy * std::sin(hdg));
        s_local = std::clamp(ds, 0.0, seg_length);
      } else if (ref_line_.type[seg_idx] == GeometryType::kArc) {
        double dx = pose.x - ref_line_.x[seg_idx];
        double dy = pose.y - ref_line_.y[seg_idx];
        double hdg = ref_line_.hdg[seg_idx];
        double curvature = arc_curvature_[ref_line_.type_index[seg_idx]];
        if (std::abs(curvature) < 1e-12) {
          s_local = std::clamp((dx * std::cos(hdg)) + (dy * std::sin(hdg)), 0.0, seg_length);
        } else {
          double radius = 1.0 / curvature;
          double center_x = ref_line_.x[seg_idx] - (radius * std::sin(hdg));
          double center_y = ref_line_.y[seg_idx] + (radius * std::cos(hdg));
          double qdx = pose.x - center_x;
          double qdy = pose.y - center_y;
          double angle_query = std::atan2(qdy, qdx);
          double angle_start = std::atan2(ref_line_.y[seg_idx] - center_y, ref_line_.x[seg_idx] - center_x);
          double delta_angle = angle_query - angle_start;
          if (curvature > 0.0) {
            while (delta_angle < 0.0) {
              delta_angle += 2.0 * M_PI;
            }
            while (delta_angle >= 2.0 * M_PI) {
              delta_angle -= 2.0 * M_PI;
            }
          } else {
            while (delta_angle > 0.0) {
              delta_angle -= 2.0 * M_PI;
            }
            while (delta_angle <= -2.0 * M_PI) {
              delta_angle += 2.0 * M_PI;
            }
          }
          s_local = std::clamp(delta_angle / curvature, 0.0, seg_length);
        }
      } else {
        s_local = ProjectToGenericSegment(ref_line_, arc_curvature_, seg_idx, seg_length, pose.x, pose.y);
      }

      double rx = 0.0;
      double ry = 0.0;
      double r_hdg = 0.0;
      EvaluateReferenceLine(ref_line_, arc_curvature_, seg_idx, ref_line_.s_offset[seg_idx] + s_local, rx, ry, r_hdg);

      double dx = pose.x - rx;
      double dy = pose.y - ry;
      double dist_sq = (dx * dx) + (dy * dy);
      if (dist_sq < min_dist_sq) {
        min_dist_sq = dist_sq;
        best_s = ref_line_.s_offset[seg_idx] + s_local;
        best_t = (-dx * std::sin(r_hdg)) + (dy * std::cos(r_hdg));
        best_rhdg = r_hdg;
      }
    }

    double elev = 0.0;
    double natural_pitch = 0.0;
    double natural_roll = 0.0;
    EvaluateNaturalOrientationAndElev(polynomials_, road_elevation_first_idx_, road_elevation_count_,
                                      road_superelevation_first_idx_, road_superelevation_count_, road_css_, road_idx,
                                      best_s, elev, natural_pitch, natural_roll);

    uint32_t best_seg_idx = first_seg;
    for (uint32_t i = 0; i < seg_count; ++i) {
      uint32_t cur_seg = first_seg + i;
      if (best_s >= ref_line_.s_offset[cur_seg]) {
        best_seg_idx = cur_seg;
      } else {
        break;
      }
    }
    double rx = 0.0;
    double ry = 0.0;
    double r_hdg = 0.0;
    EvaluateReferenceLine(ref_line_, arc_curvature_, best_seg_idx, best_s, rx, ry, r_hdg);

    double dx = pose.x - rx;
    double dy = pose.y - ry;
    double dz = pose.z - elev;

    // Base roll calculation
    Matrix3x3 r_road_base = EulerToMatrix(best_rhdg, natural_pitch, natural_roll);
    double road_t_base = (r_road_base[0][1] * dx) + (r_road_base[1][1] * dy) + (r_road_base[2][1] * dz);

    // Shape evaluation and roll correction
    double shape_grad = EvaluateShapeTGradient(shapes_, road_shape_first_idx_[road_idx], road_shape_count_[road_idx],
                                               best_s, road_t_base);
    double roll_total = natural_roll + std::atan(shape_grad);

    Matrix3x3 r_road = EulerToMatrix(best_rhdg, natural_pitch, roll_total);
    double road_t = (r_road[0][1] * dx) + (r_road[1][1] * dy) + (r_road[2][1] * dz);

    double t_left = 0.0;
    double t_right = 0.0;
    GetRoadWidthLimits(road_idx, best_s, t_left, t_right);
    constexpr double kSnappingTolerance = 5.0;

    if (road_t >= t_right - kSnappingTolerance && road_t <= t_left + kSnappingTolerance) {
      RoadPose road_pose;
      road_pose.road = static_cast<RoadId>(road_idx);
      road_pose.s = best_s;
      road_pose.t = road_t;

      double h_surf = 0.0;
      EvaluateCrossSectionSurfaceOffset(polynomials_, strips_, road_css_, road_idx, best_s, road_t, h_surf);

      double h_shape =
          EvaluateShapeHeight(shapes_, road_shape_first_idx_[road_idx], road_shape_count_[road_idx], best_s, road_t);

      double local_h = (r_road[0][2] * dx) + (r_road[1][2] * dy) + (r_road[2][2] * dz);
      road_pose.h = local_h - h_surf - h_shape;

      Matrix3x3 r_inertial = EulerToMatrix(pose.heading, pose.pitch, pose.roll);
      Matrix3x3 r_road_t = TransposeMatrix(r_road);
      Matrix3x3 r_offset = ComposeRotations(r_road_t, r_inertial);
      EulerAngles offset_angles = MatrixToEuler(r_offset);
      road_pose.heading = offset_angles.heading;
      road_pose.pitch = offset_angles.pitch;
      road_pose.roll = offset_angles.roll;

      return road_pose;
    }
    return std::nullopt;
  };

  // 1. Check temporal coherence fast path
  if (ctx.last_road.has_value()) {
    auto road_idx = static_cast<uint32_t>(*ctx.last_road);
    auto fast_pose = snap_to_road(road_idx);
    if (fast_pose.has_value()) {
      auto first_seg = road_ref_line_first_idx_[road_idx];
      auto seg_count = road_ref_line_count_[road_idx];
      for (uint32_t i = 0; i < seg_count; ++i) {
        auto idx = first_seg + i;
        double s_start = ref_line_.s_offset[idx];
        double s_end = s_start + ref_line_.length[idx];
        if (fast_pose->s >= s_start && fast_pose->s <= s_end) {
          ctx.last_segment_idx = idx;
          break;
        }
      }
      return fast_pose;
    }
  }

  // 2. Traversal stack-based BVH search
  if (bvh_nodes_.empty()) {
    return std::nullopt;
  }

  std::array<uint32_t, 64> stack{};
  int stack_ptr = 0;
  stack[stack_ptr++] = 0;

  double min_t_distance = std::numeric_limits<double>::max();
  std::optional<RoadPose> best_overall_pose;

  while (stack_ptr > 0) {
    auto curr_idx = stack[--stack_ptr];
    const auto& node = bvh_nodes_[curr_idx];

    double dist_to_box = DistancePointToAabb(pose.x, pose.y, node.min_x, node.min_y, node.max_x, node.max_y);
    if (dist_to_box > min_t_distance) {
      continue;
    }

    bool is_leaf = (node.right & 0x80000000) != 0;
    if (is_leaf) {
      auto prim_start = node.left;
      auto prim_count = node.right & 0x7FFFFFFF;

      for (uint32_t i = 0; i < prim_count; ++i) {
        const auto& prim = bvh_primitives_[prim_start + i];
        auto candidate = snap_to_road(prim.road_idx);
        if (candidate.has_value()) {
          double abs_t = std::abs(candidate->t);
          if (abs_t < min_t_distance) {
            min_t_distance = abs_t;
            best_overall_pose = candidate;
          }
        }
      }
    } else {
      auto left_child = node.left;
      auto right_child = node.right & 0x7FFFFFFF;

      double dist_left = DistancePointToAabb(pose.x, pose.y, bvh_nodes_[left_child].min_x, bvh_nodes_[left_child].min_y,
                                             bvh_nodes_[left_child].max_x, bvh_nodes_[left_child].max_y);
      double dist_right =
          DistancePointToAabb(pose.x, pose.y, bvh_nodes_[right_child].min_x, bvh_nodes_[right_child].min_y,
                              bvh_nodes_[right_child].max_x, bvh_nodes_[right_child].max_y);

      if (dist_left < dist_right) {
        stack[stack_ptr++] = right_child;
        stack[stack_ptr++] = left_child;
      } else {
        stack[stack_ptr++] = left_child;
        stack[stack_ptr++] = right_child;
      }
    }
  }

  if (best_overall_pose.has_value()) {
    ctx.last_road = best_overall_pose->road;
    auto road_idx = static_cast<uint32_t>(best_overall_pose->road);
    auto first_seg = road_ref_line_first_idx_[road_idx];
    auto seg_count = road_ref_line_count_[road_idx];
    for (uint32_t i = 0; i < seg_count; ++i) {
      auto idx = first_seg + i;
      double s_start = ref_line_.s_offset[idx];
      double s_end = s_start + ref_line_.length[idx];
      if (best_overall_pose->s >= s_start && best_overall_pose->s <= s_end) {
        ctx.last_segment_idx = idx;
        break;
      }
    }
  }

  return best_overall_pose;
}

auto CompiledPhysicsModel::InertialToLane(InertialPose pose, QueryContext& ctx) const noexcept
    -> std::optional<LanePose> {
  auto road_pose_opt = InertialToRoad(pose, ctx);
  if (!road_pose_opt.has_value()) {
    return std::nullopt;
  }
  return RoadToLane(*road_pose_opt, ctx);
}

auto CompiledPhysicsModel::RoadToLane(RoadPose pose, QueryContext& ctx) const noexcept -> std::optional<LanePose> {
  auto road_idx = static_cast<uint32_t>(pose.road);
  if (road_idx >= road_string_ids_.size()) {
    return std::nullopt;
  }

  auto road_sec_first = road_section_first_idx_[road_idx];
  auto road_sec_count = road_section_count_[road_idx];
  if (road_sec_count == 0) {
    return std::nullopt;
  }

  // Find the active lane section at pose.s
  auto sec_idx = road_sec_first;
  for (uint32_t i = 0; i < road_sec_count; ++i) {
    auto cur_sec = road_sec_first + i;
    if (pose.s >= section_s_[cur_sec]) {
      sec_idx = cur_sec;
    } else {
      break;
    }
  }

  auto first_lane_in_sec = section_first_lane_idx_[sec_idx];
  auto lane_cnt_in_sec = section_lane_count_[sec_idx];

  // Calculate road-level lane offset
  double lane_offset_val = 0.0;
  auto lo_first = road_lane_offset_first_idx_[road_idx];
  auto lo_count = road_lane_offset_count_[road_idx];
  if (lo_count > 0) {
    auto active_lo = lo_first;
    for (uint32_t i = 0; i < lo_count; ++i) {
      auto cur_lo = lo_first + i;
      if (pose.s >= lane_offset_s_start_[cur_lo]) {
        active_lo = cur_lo;
      } else {
        break;
      }
    }
    double ds_lo = pose.s - lane_offset_s_start_[active_lo];
    lane_offset_val =
        lane_offset_a_[active_lo] +
        (ds_lo * (lane_offset_b_[active_lo] + ds_lo * (lane_offset_c_[active_lo] + ds_lo * lane_offset_d_[active_lo])));
  }

  double t_relative = pose.t - lane_offset_val;

  uint32_t matched_lane_idx = 0;
  bool found = false;
  double t_center = 0.0;
  double w_target = 0.0;
  int target_id = 0;

  if (t_relative > 0.0) {
    // Left lanes: IDs > 0, sorted ascending (e.g. 1, 2, 3...)
    double t_inner = 0.0;
    for (uint32_t i = 0; i < lane_cnt_in_sec; ++i) {
      uint32_t lane_idx = first_lane_in_sec + i;
      int lane_id = lane_original_id_[lane_idx];
      if (lane_id <= 0) {
        continue;
      }
      double w = LaneWidth(static_cast<LaneId>(lane_idx), pose.s);
      double t_outer = t_inner + w;
      if (w > 0.0 && t_relative >= t_inner && t_relative <= t_outer) {
        matched_lane_idx = lane_idx;
        t_center = t_inner + (0.5 * w);
        w_target = w;
        target_id = lane_id;
        found = true;
        break;
      }
      t_inner = t_outer;
    }
  } else if (t_relative < 0.0) {
    // Right lanes: IDs < 0, sorted ascending (e.g. -3, -2, -1)
    // Walk them in reverse order (from -1 down to -3) to go from inside to outside.
    double t_inner = 0.0;
    for (int i = static_cast<int>(lane_cnt_in_sec) - 1; i >= 0; --i) {
      uint32_t lane_idx = first_lane_in_sec + static_cast<uint32_t>(i);
      int lane_id = lane_original_id_[lane_idx];
      if (lane_id >= 0) {
        continue;
      }
      double w = LaneWidth(static_cast<LaneId>(lane_idx), pose.s);
      double t_outer = t_inner - w;
      if (w > 0.0 && t_relative <= t_inner && t_relative >= t_outer) {
        matched_lane_idx = lane_idx;
        t_center = t_inner - (0.5 * w);
        w_target = w;
        target_id = lane_id;
        found = true;
        break;
      }
      t_inner = t_outer;
    }
  }

  if (!found) {
    return std::nullopt;
  }

  // Evaluate lane height offset
  double h_inner = 0.0;
  double h_outer = 0.0;
  uint32_t h_first = lane_first_height_idx_[matched_lane_idx];
  uint32_t h_count = lane_height_count_[matched_lane_idx];
  if (h_count > 0) {
    uint32_t active_h = h_first;
    for (uint32_t i = 0; i < h_count; ++i) {
      uint32_t cur_h = h_first + i;
      if (pose.s >= lane_height_s_start_[cur_h]) {
        active_h = cur_h;
      } else {
        break;
      }
    }
    h_inner = lane_height_inner_[active_h];
    h_outer = lane_height_outer_[active_h];
  }

  double t_lane = t_relative - t_center;
  double f = 0.0;
  if (w_target > 0.0) {
    if (target_id > 0) {
      f = 0.5 + (t_lane / w_target);
    } else if (target_id < 0) {
      f = 0.5 - (t_lane / w_target);
    }
  }
  f = std::clamp(f, 0.0, 1.0);
  double h_offset = h_inner + (f * (h_outer - h_inner));

  LanePose lane_pose;
  lane_pose.s = pose.s;
  lane_pose.t = t_lane;
  lane_pose.h = pose.h - h_offset;
  lane_pose.heading = pose.heading;
  lane_pose.pitch = pose.pitch;
  lane_pose.roll = pose.roll;
  lane_pose.road = pose.road;
  lane_pose.lane = static_cast<LaneId>(matched_lane_idx);

  // Update query context road cache
  ctx.last_road = pose.road;

  return lane_pose;
}

void CompiledPhysicsModel::GetRoadWidthLimits(uint32_t road_idx, double s_coord, double& t_left,
                                              double& t_right) const noexcept {
  t_left = 0.0;
  t_right = 0.0;

  auto road_sec_first = road_section_first_idx_[road_idx];
  auto road_sec_count = road_section_count_[road_idx];
  if (road_sec_count == 0) {
    return;
  }

  auto sec_idx = road_sec_first;
  for (uint32_t i = 0; i < road_sec_count; ++i) {
    auto cur_sec = road_sec_first + i;
    if (s_coord >= section_s_[cur_sec]) {
      sec_idx = cur_sec;
    } else {
      break;
    }
  }

  auto first_lane_in_sec = section_first_lane_idx_[sec_idx];
  auto lane_cnt_in_sec = section_lane_count_[sec_idx];

  for (uint32_t i = 0; i < lane_cnt_in_sec; ++i) {
    auto lane_idx = first_lane_in_sec + i;
    auto lane_id = lane_original_id_[lane_idx];
    double w = LaneWidth(static_cast<LaneId>(lane_idx), s_coord);
    if (lane_id > 0) {
      t_left += w;
    } else if (lane_id < 0) {
      t_right -= w;
    }
  }

  double lane_offset_val = 0.0;
  auto lo_first = road_lane_offset_first_idx_[road_idx];
  auto lo_count = road_lane_offset_count_[road_idx];
  if (lo_count > 0) {
    auto active_lo = lo_first;
    for (uint32_t i = 0; i < lo_count; ++i) {
      auto cur_lo = lo_first + i;
      if (s_coord >= lane_offset_s_start_[cur_lo]) {
        active_lo = cur_lo;
      } else {
        break;
      }
    }
    double ds_lo = s_coord - lane_offset_s_start_[active_lo];
    lane_offset_val =
        lane_offset_a_[active_lo] +
        (ds_lo * (lane_offset_b_[active_lo] + ds_lo * (lane_offset_c_[active_lo] + ds_lo * lane_offset_d_[active_lo])));
  }

  t_left += lane_offset_val;
  t_right += lane_offset_val;
}

auto CompiledPhysicsModel::LaneToRoad(LanePose pose, QueryContext& /*ctx*/) const noexcept -> RoadPose {
  auto lane_idx = static_cast<uint32_t>(pose.lane);
  if (lane_idx >= lane_original_id_.size()) {
    return RoadPose{};
  }

  double s = pose.s;
  int target_id = lane_original_id_[lane_idx];
  RoadId road_id = lane_road_id_[lane_idx];
  auto road_idx = static_cast<uint32_t>(road_id);
  uint32_t sec_idx = lane_section_idx_[lane_idx];

  // 1. Compute cumulative inner boundary width
  double inner_boundary_t = 0.0;
  uint32_t first_lane_in_sec = section_first_lane_idx_[sec_idx];
  uint32_t lane_cnt_in_sec = section_lane_count_[sec_idx];

  for (uint32_t i = 0; i < lane_cnt_in_sec; ++i) {
    uint32_t other_idx = first_lane_in_sec + i;
    int other_id = lane_original_id_[other_idx];
    if (target_id > 0) {
      if (other_id > 0 && other_id < target_id) {
        inner_boundary_t += LaneWidth(static_cast<LaneId>(other_idx), s);
      }
    } else if (target_id < 0) {
      if (other_id < 0 && other_id > target_id) {
        inner_boundary_t += LaneWidth(static_cast<LaneId>(other_idx), s);
      }
    }
  }

  if (target_id < 0) {
    inner_boundary_t = -inner_boundary_t;
  }

  // 2. Target lane width
  double w_target = LaneWidth(pose.lane, s);

  // 3. Center line t of the lane
  double t_center = 0.0;
  if (target_id > 0) {
    t_center = inner_boundary_t + (0.5 * w_target);
  } else if (target_id < 0) {
    t_center = inner_boundary_t - (0.5 * w_target);
  }

  double road_t = t_center + pose.t;

  // 4. Add road-level laneOffset
  double lane_offset_val = 0.0;
  uint32_t lo_first = road_lane_offset_first_idx_[road_idx];
  uint32_t lo_count = road_lane_offset_count_[road_idx];
  if (lo_count > 0) {
    uint32_t active_lo = lo_first;
    for (uint32_t i = 0; i < lo_count; ++i) {
      uint32_t cur_lo = lo_first + i;
      if (s >= lane_offset_s_start_[cur_lo]) {
        active_lo = cur_lo;
      } else {
        break;
      }
    }
    double ds_lo = s - lane_offset_s_start_[active_lo];
    lane_offset_val = lane_offset_a_[active_lo] + (lane_offset_b_[active_lo] * ds_lo) +
                      (lane_offset_c_[active_lo] * ds_lo * ds_lo) + (lane_offset_d_[active_lo] * ds_lo * ds_lo * ds_lo);
  }
  road_t += lane_offset_val;

  // 5. Evaluate lane height offset
  double h_inner = 0.0;
  double h_outer = 0.0;
  uint32_t h_first = lane_first_height_idx_[lane_idx];
  uint32_t h_count = lane_height_count_[lane_idx];
  if (h_count > 0) {
    uint32_t active_h = h_first;
    for (uint32_t i = 0; i < h_count; ++i) {
      uint32_t cur_h = h_first + i;
      if (s >= lane_height_s_start_[cur_h]) {
        active_h = cur_h;
      } else {
        break;
      }
    }
    h_inner = lane_height_inner_[active_h];
    h_outer = lane_height_outer_[active_h];
  }

  double f = 0.0;
  if (w_target > 0.0) {
    if (target_id > 0) {
      f = 0.5 + (pose.t / w_target);
    } else if (target_id < 0) {
      f = 0.5 - (pose.t / w_target);
    }
  }
  f = std::clamp(f, 0.0, 1.0);
  double h_offset = h_inner + (f * (h_outer - h_inner));
  double road_h = pose.h + h_offset;

  RoadPose road_pose;
  road_pose.s = s;
  road_pose.t = road_t;
  road_pose.h = road_h;
  road_pose.heading = pose.heading;
  road_pose.pitch = pose.pitch;
  road_pose.roll = pose.roll;
  road_pose.road = road_id;
  return road_pose;
}

auto CompiledPhysicsModel::RoadCount() const noexcept -> std::size_t { return road_string_ids_.size(); }

auto CompiledPhysicsModel::RoadIdFromString(std::string_view original_id) const noexcept -> std::optional<RoadId> {
  auto find_it = std::ranges::find(road_string_ids_, original_id);
  if (find_it != road_string_ids_.end()) {
    return static_cast<RoadId>(std::distance(road_string_ids_.begin(), find_it));
  }
  return std::nullopt;
}

auto CompiledPhysicsModel::OriginalRoadId(RoadId road_id) const noexcept -> std::string_view {
  auto idx = static_cast<uint32_t>(road_id);
  if (idx < road_string_ids_.size()) {
    return road_string_ids_[idx];
  }
  return "";
}

auto CompiledPhysicsModel::RoadLength(RoadId road_id) const noexcept -> double {
  auto idx = static_cast<uint32_t>(road_id);
  if (idx < road_lengths_.size()) {
    return road_lengths_[idx];
  }
  return 0.0;
}

auto CompiledPhysicsModel::LaneCount() const noexcept -> std::size_t { return lane_original_id_.size(); }

auto CompiledPhysicsModel::LaneRoad(LaneId lane_id) const noexcept -> RoadId {
  auto idx = static_cast<uint32_t>(lane_id);
  if (idx < lane_road_id_.size()) {
    return lane_road_id_[idx];
  }
  return RoadId{0};
}

auto CompiledPhysicsModel::OriginalLaneId(LaneId lane_id) const noexcept -> int {
  auto idx = static_cast<uint32_t>(lane_id);
  if (idx < lane_original_id_.size()) {
    return lane_original_id_[idx];
  }
  return 0;
}

auto CompiledPhysicsModel::LaneWidth(LaneId lane_id, double s_coord) const noexcept -> double {
  auto idx = static_cast<uint32_t>(lane_id);
  if (idx >= lane_original_id_.size()) {
    return 0.0;
  }
  uint32_t w_first = lane_first_width_idx_[idx];
  uint32_t w_count = lane_width_count_[idx];
  if (w_count == 0) {
    return 0.0;
  }

  uint32_t active_idx = w_first;
  for (uint32_t i = 0; i < w_count; ++i) {
    uint32_t cur_idx = w_first + i;
    if (s_coord >= lane_width_s_start_[cur_idx]) {
      active_idx = cur_idx;
    } else {
      break;
    }
  }

  double ds = s_coord - lane_width_s_start_[active_idx];
  return lane_width_a_[active_idx] + (lane_width_b_[active_idx] * ds) + (lane_width_c_[active_idx] * ds * ds) +
         (lane_width_d_[active_idx] * ds * ds * ds);
}

auto BuildCompiledPhysicsModel(const ast::AbstractSyntaxTree& map) -> CompiledPhysicsModel {
  CompiledPhysicsModel model;

  for (const auto& road : map.roads) {
    model.road_string_ids_.push_back(road.id);
    model.road_lengths_.push_back(road.length);

    // Reference line compilation
    model.road_ref_line_first_idx_.push_back(static_cast<uint32_t>(model.ref_line_.s_offset.size()));
    model.road_ref_line_count_.push_back(static_cast<uint32_t>(road.plan_view.size()));

    for (const auto& geom : road.plan_view) {
      model.ref_line_.s_offset.push_back(geom.s);
      model.ref_line_.length.push_back(geom.length);
      model.ref_line_.x.push_back(geom.x);
      model.ref_line_.y.push_back(geom.y);
      model.ref_line_.hdg.push_back(geom.hdg);

      if (std::holds_alternative<ast::Line>(geom.shape)) {
        model.ref_line_.type.push_back(GeometryType::kLine);
        model.ref_line_.type_index.push_back(0);
        model.ref_line_.spiral_curv_start.push_back(0.0);
        model.ref_line_.spiral_curv_end.push_back(0.0);
        model.ref_line_.pp3_a_u.push_back(0.0);
        model.ref_line_.pp3_b_u.push_back(0.0);
        model.ref_line_.pp3_c_u.push_back(0.0);
        model.ref_line_.pp3_d_u.push_back(0.0);
        model.ref_line_.pp3_a_v.push_back(0.0);
        model.ref_line_.pp3_b_v.push_back(0.0);
        model.ref_line_.pp3_c_v.push_back(0.0);
        model.ref_line_.pp3_d_v.push_back(0.0);
        model.ref_line_.pp3_p_range.push_back(0);
      } else if (std::holds_alternative<ast::Arc>(geom.shape)) {
        model.ref_line_.type.push_back(GeometryType::kArc);
        model.ref_line_.type_index.push_back(static_cast<uint32_t>(model.arc_curvature_.size()));
        model.arc_curvature_.push_back(std::get<ast::Arc>(geom.shape).curvature);
        model.ref_line_.spiral_curv_start.push_back(0.0);
        model.ref_line_.spiral_curv_end.push_back(0.0);
        model.ref_line_.pp3_a_u.push_back(0.0);
        model.ref_line_.pp3_b_u.push_back(0.0);
        model.ref_line_.pp3_c_u.push_back(0.0);
        model.ref_line_.pp3_d_u.push_back(0.0);
        model.ref_line_.pp3_a_v.push_back(0.0);
        model.ref_line_.pp3_b_v.push_back(0.0);
        model.ref_line_.pp3_c_v.push_back(0.0);
        model.ref_line_.pp3_d_v.push_back(0.0);
        model.ref_line_.pp3_p_range.push_back(0);
      } else if (std::holds_alternative<ast::Spiral>(geom.shape)) {
        model.ref_line_.type.push_back(GeometryType::kSpiral);
        model.ref_line_.type_index.push_back(0);
        const auto& spiral = std::get<ast::Spiral>(geom.shape);
        model.ref_line_.spiral_curv_start.push_back(spiral.curv_start);
        model.ref_line_.spiral_curv_end.push_back(spiral.curv_end);
        model.ref_line_.pp3_a_u.push_back(0.0);
        model.ref_line_.pp3_b_u.push_back(0.0);
        model.ref_line_.pp3_c_u.push_back(0.0);
        model.ref_line_.pp3_d_u.push_back(0.0);
        model.ref_line_.pp3_a_v.push_back(0.0);
        model.ref_line_.pp3_b_v.push_back(0.0);
        model.ref_line_.pp3_c_v.push_back(0.0);
        model.ref_line_.pp3_d_v.push_back(0.0);
        model.ref_line_.pp3_p_range.push_back(0);
      } else if (std::holds_alternative<ast::Poly3>(geom.shape)) {
        model.ref_line_.type.push_back(GeometryType::kParamPoly3);
        model.ref_line_.type_index.push_back(0);
        model.ref_line_.spiral_curv_start.push_back(0.0);
        model.ref_line_.spiral_curv_end.push_back(0.0);
        const auto& poly = std::get<ast::Poly3>(geom.shape);
        ast::ParamPoly3 param = ConvertPoly3ToParamPoly3(geom.length, poly.a, poly.b, poly.c, poly.d);
        model.ref_line_.pp3_a_u.push_back(param.a_u);
        model.ref_line_.pp3_b_u.push_back(param.b_u);
        model.ref_line_.pp3_c_u.push_back(param.c_u);
        model.ref_line_.pp3_d_u.push_back(param.d_u);
        model.ref_line_.pp3_a_v.push_back(param.a_v);
        model.ref_line_.pp3_b_v.push_back(param.b_v);
        model.ref_line_.pp3_c_v.push_back(param.c_v);
        model.ref_line_.pp3_d_v.push_back(param.d_v);
        model.ref_line_.pp3_p_range.push_back(1);
      } else if (std::holds_alternative<ast::ParamPoly3>(geom.shape)) {
        model.ref_line_.type.push_back(GeometryType::kParamPoly3);
        model.ref_line_.type_index.push_back(0);
        model.ref_line_.spiral_curv_start.push_back(0.0);
        model.ref_line_.spiral_curv_end.push_back(0.0);
        const auto& param = std::get<ast::ParamPoly3>(geom.shape);
        model.ref_line_.pp3_a_u.push_back(param.a_u);
        model.ref_line_.pp3_b_u.push_back(param.b_u);
        model.ref_line_.pp3_c_u.push_back(param.c_u);
        model.ref_line_.pp3_d_u.push_back(param.d_u);
        model.ref_line_.pp3_a_v.push_back(param.a_v);
        model.ref_line_.pp3_b_v.push_back(param.b_v);
        model.ref_line_.pp3_c_v.push_back(param.c_v);
        model.ref_line_.pp3_d_v.push_back(param.d_v);
        model.ref_line_.pp3_p_range.push_back(param.p_range == ast::PRange::kArcLength ? 1 : 0);
      }
    }

    // Elevation profile compilation
    {
      auto first_idx = 0U;
      auto count = 0U;
      std::vector<ast::Coefficient> coeffs;
      coeffs.reserve(road.elevation_profile.elevations.size());
      for (const auto& elev : road.elevation_profile.elevations) {
        coeffs.push_back({elev.s, elev.a, elev.b, elev.c, elev.d});
      }
      CompileCoefficients(coeffs, model.polynomials_, first_idx, count);
      model.road_elevation_first_idx_.push_back(first_idx);
      model.road_elevation_count_.push_back(count);
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
      CompileCoefficients(coeffs, model.polynomials_, first_idx, count);
      model.road_superelevation_first_idx_.push_back(first_idx);
      model.road_superelevation_count_.push_back(count);
    }

    // Shape profile compilation
    {
      auto first_idx = static_cast<uint32_t>(model.shapes_.s.size());
      auto count = static_cast<uint32_t>(road.lateral_profile.shapes.size());
      for (const auto& shape : road.lateral_profile.shapes) {
        model.shapes_.s.push_back(shape.s);
        model.shapes_.t.push_back(shape.t);
        model.shapes_.a.push_back(shape.a);
        model.shapes_.b.push_back(shape.b);
        model.shapes_.c.push_back(shape.c);
        model.shapes_.d.push_back(shape.d);
      }
      model.road_shape_first_idx_.push_back(first_idx);
      model.road_shape_count_.push_back(count);
    }

    const auto& css_opt = road.lateral_profile.cross_section_surface;
    if (css_opt.has_value()) {
      const auto& css = *css_opt;

      model.road_css_.first_strip_idx.push_back(static_cast<uint32_t>(model.strips_.strip_id.size()));
      model.road_css_.strip_count.push_back(static_cast<uint32_t>(css.strips.size()));

      auto t_off_idx = 0U;
      auto t_off_cnt = 0U;
      CompileCoefficients(css.t_offset, model.polynomials_, t_off_idx, t_off_cnt);
      model.road_css_.t_offset_first_idx.push_back(t_off_idx);
      model.road_css_.t_offset_count.push_back(t_off_cnt);

      auto sorted_strips = css.strips;
      std::ranges::sort(sorted_strips, [](const auto& val_a, const auto& val_b) noexcept -> bool {
        return std::abs(val_a.id) < std::abs(val_b.id);
      });

      for (const auto& strip : sorted_strips) {
        model.strips_.strip_id.push_back(strip.id);
        model.strips_.is_relative.push_back(static_cast<uint8_t>(strip.mode == "relative"));

        auto first_idx = 0U;
        auto count = 0U;

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

    // Lane offset profile compilation (road level)
    {
      auto first_idx = static_cast<uint32_t>(model.lane_offset_s_start_.size());
      auto count = static_cast<uint32_t>(road.lanes.offsets.size());
      for (const auto& offset : road.lanes.offsets) {
        model.lane_offset_s_start_.push_back(offset.s);
        model.lane_offset_a_.push_back(offset.a);
        model.lane_offset_b_.push_back(offset.b);
        model.lane_offset_c_.push_back(offset.c);
        model.lane_offset_d_.push_back(offset.d);
      }
      model.road_lane_offset_first_idx_.push_back(first_idx);
      model.road_lane_offset_count_.push_back(count);
    }

    // Lane sections compilation
    {
      auto road_sec_first = static_cast<uint32_t>(model.section_s_.size());
      auto road_sec_count = static_cast<uint32_t>(road.lanes.sections.size());

      for (const auto& section : road.lanes.sections) {
        model.section_s_.push_back(section.s);

        // Accumulate all lanes in this section: right, center, left.
        // We will sort them by ID ascending.
        std::vector<ast::Lane> sorted_section_lanes;
        sorted_section_lanes.reserve(section.right.size() + section.center.size() + section.left.size());
        for (const auto& lane : section.right) {
          sorted_section_lanes.push_back(lane);
        }
        for (const auto& lane : section.center) {
          sorted_section_lanes.push_back(lane);
        }
        for (const auto& lane : section.left) {
          sorted_section_lanes.push_back(lane);
        }

        // Sort by ID ascending
        std::ranges::sort(sorted_section_lanes, [](const auto& lhs_lane, const auto& rhs_lane) noexcept -> bool {
          return lhs_lane.id < rhs_lane.id;
        });

        auto section_first_lane = static_cast<uint32_t>(model.lane_original_id_.size());
        auto section_lane_count = static_cast<uint32_t>(sorted_section_lanes.size());

        model.section_first_lane_idx_.push_back(section_first_lane);
        model.section_lane_count_.push_back(section_lane_count);

        // Now compile each lane in this section
        for (const auto& lane : sorted_section_lanes) {
          model.lane_original_id_.push_back(lane.id);
          model.lane_road_id_.push_back(static_cast<RoadId>(model.road_string_ids_.size() - 1));
          model.lane_section_idx_.push_back(static_cast<uint32_t>(model.section_s_.size() - 1));

          // Width polynomials for this lane
          auto w_first = static_cast<uint32_t>(model.lane_width_s_start_.size());
          auto w_count = static_cast<uint32_t>(lane.widths.size());
          for (const auto& width_poly : lane.widths) {
            // Note: sOffset is relative to section start s.
            model.lane_width_s_start_.push_back(section.s + width_poly.s_offset);
            model.lane_width_a_.push_back(width_poly.a);
            model.lane_width_b_.push_back(width_poly.b);
            model.lane_width_c_.push_back(width_poly.c);
            model.lane_width_d_.push_back(width_poly.d);
          }
          model.lane_first_width_idx_.push_back(w_first);
          model.lane_width_count_.push_back(w_count);

          // Height polynomials for this lane
          auto h_first = static_cast<uint32_t>(model.lane_height_s_start_.size());
          auto h_count = static_cast<uint32_t>(lane.heights.size());
          for (const auto& height_poly : lane.heights) {
            // sOffset is relative to section start s.
            model.lane_height_s_start_.push_back(section.s + height_poly.s_offset);
            model.lane_height_inner_.push_back(height_poly.inner);
            model.lane_height_outer_.push_back(height_poly.outer);
          }
          model.lane_first_height_idx_.push_back(h_first);
          model.lane_height_count_.push_back(h_count);
        }
      }
      model.road_section_first_idx_.push_back(road_sec_first);
      model.road_section_count_.push_back(road_sec_count);
    }
  }

  // Global BVH construction
  std::vector<double> road_max_t;
  road_max_t.reserve(map.roads.size());

  for (const auto& road : map.roads) {
    double max_road_t = 0.0;
    for (size_t sec_idx = 0; sec_idx < road.lanes.sections.size(); ++sec_idx) {
      const auto& section = road.lanes.sections[sec_idx];
      double sec_length = 0.0;
      if (sec_idx + 1 < road.lanes.sections.size()) {
        sec_length = road.lanes.sections[sec_idx + 1].s - section.s;
      } else {
        sec_length = road.length - section.s;
      }

      constexpr int kSecSamples = 10;
      for (int i = 0; i <= kSecSamples; ++i) {
        double s_local = (static_cast<double>(i) / kSecSamples) * sec_length;
        double left_width = 0.0;
        double right_width = 0.0;

        for (const auto& lane : section.left) {
          left_width += EvaluateAstLaneWidth(lane, s_local);
        }
        for (const auto& lane : section.right) {
          right_width += EvaluateAstLaneWidth(lane, s_local);
        }

        // Evaluate road laneOffset
        double lane_offset_val = 0.0;
        if (!road.lanes.offsets.empty()) {
          const ast::LaneOffset* active = road.lanes.offsets.data();
          double s_road = section.s + s_local;
          for (const auto& offset : road.lanes.offsets) {
            if (s_road >= offset.s) {
              active = &offset;
            } else {
              break;
            }
          }
          double ds_offset = s_road - active->s;
          lane_offset_val = active->a + (ds_offset * (active->b + (ds_offset * (active->c + (ds_offset * active->d)))));
        }

        max_road_t = std::max(max_road_t, left_width + std::abs(lane_offset_val));
        max_road_t = std::max(max_road_t, right_width + std::abs(lane_offset_val));
      }
    }
    constexpr double kRoadWidthSafetyBuffer = 0.1;
    max_road_t += kRoadWidthSafetyBuffer;
    road_max_t.push_back(max_road_t);
  }

  std::vector<BvhPrimitiveInfo> temp_primitives;
  std::vector<Aabb> temp_aabbs;

  auto num_roads = static_cast<uint32_t>(model.road_lengths_.size());
  for (uint32_t road_idx = 0; road_idx < num_roads; ++road_idx) {
    auto first_seg = model.road_ref_line_first_idx_[road_idx];
    auto seg_count = model.road_ref_line_count_[road_idx];
    double inflation = road_max_t[road_idx];
    for (uint32_t i = 0; i < seg_count; ++i) {
      uint32_t seg_idx = first_seg + i;
      temp_primitives.push_back(BvhPrimitiveInfo{.road_idx = road_idx, .segment_idx = seg_idx});
      temp_aabbs.push_back(ComputeSegmentAabb(model.ref_line_, model.arc_curvature_, seg_idx, inflation));
    }
  }

  if (!temp_primitives.empty()) {
    std::vector<uint32_t> prim_indices(temp_primitives.size());
    for (uint32_t i = 0; i < prim_indices.size(); ++i) {
      prim_indices[i] = i;
    }
    BuildBvhRecursive(model.bvh_nodes_, model.bvh_primitives_, prim_indices, temp_primitives, temp_aabbs, 0,
                      static_cast<uint32_t>(temp_primitives.size()));
  }

  return model;
}

}  // namespace strada::cpm
