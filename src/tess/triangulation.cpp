// SPDX-License-Identifier: BSL-1.0

#include "triangulation.hpp"

namespace strada::tess {

auto TriangulatePolygon(const std::vector<Vertex>& vertices) -> std::vector<std::uint32_t> {
  std::vector<std::uint32_t> indices;
  if (vertices.size() < 3) {
    return indices;
  }

  size_t n = vertices.size();
  std::vector<std::uint32_t> v(n);

  // Compute signed area to determine winding order
  double area = 0.0;
  for (size_t i = 0; i < n; ++i) {
    const auto& p1 = vertices[i];
    const auto& p2 = vertices[(i + 1) % n];
    area += (static_cast<double>(p1.x) * p2.y) - (static_cast<double>(p2.x) * p1.y);
  }

  // Winding order: if area is negative, we want CW, if positive CCW.
  if (area < 0.0) {
    for (size_t i = 0; i < n; ++i) {
      v[i] = static_cast<std::uint32_t>(n - 1 - i);
    }
  } else {
    for (size_t i = 0; i < n; ++i) {
      v[i] = static_cast<std::uint32_t>(i);
    }
  }

  // Helper functions inside TriangulatePolygon
  auto inside_triangle = [](float ax, float ay, float bx, float by, float cx, float cy, float px, float py) -> bool {
    float ax_px = ax - px;
    float ay_py = ay - py;
    float bx_px = bx - px;
    float by_py = by - py;
    float cx_px = cx - px;
    float cy_py = cy - py;

    float ccw_ab = ax_px * by_py - ay_py * bx_px;
    float ccw_bc = bx_px * cy_py - by_py * cx_px;
    float ccw_ca = cx_px * ay_py - cy_py * ax_px;

    return (ccw_ab >= 0.0f && ccw_bc >= 0.0f && ccw_ca >= 0.0f) || (ccw_ab <= 0.0f && ccw_bc <= 0.0f && ccw_ca <= 0.0f);
  };

  auto is_ear = [&](size_t u, size_t w, size_t cv, const std::vector<std::uint32_t>& v_indices) -> bool {
    const auto& a = vertices[v_indices[u]];
    const auto& b = vertices[v_indices[w]];
    const auto& c = vertices[v_indices[cv]];

    // Check if triangle is convex (CCW)
    float cross_product = (b.x - a.x) * (c.y - b.y) - (b.y - a.y) * (c.x - b.x);
    if (cross_product <= 0.0f) {
      return false;
    }

    // Check if any other vertex is inside the triangle
    for (size_t p = 0; p < v_indices.size(); ++p) {
      if (p == u || p == w || p == cv) {
        continue;
      }
      const auto& pt = vertices[v_indices[p]];
      if (inside_triangle(a.x, a.y, b.x, b.y, c.x, c.y, pt.x, pt.y)) {
        return false;
      }
    }
    return true;
  };

  size_t count = 2 * n;  // Prevent infinite loop on degenerate polygons
  size_t nv = n;
  while (nv > 2) {
    if (count == 0) {
      // Degenerate fallback to triangle fan to avoid hanging
      for (size_t i = 1; i < nv - 1; ++i) {
        indices.push_back(v[0]);
        indices.push_back(v[i]);
        indices.push_back(v[i + 1]);
      }
      return indices;
    }
    count--;

    for (size_t i = 0; i < nv; ++i) {
      size_t u = (i == 0) ? (nv - 1) : (i - 1);
      size_t w = i;
      size_t cv = (i + 1 == nv) ? 0 : (i + 1);

      if (is_ear(u, w, cv, v)) {
        indices.push_back(v[u]);
        indices.push_back(v[w]);
        indices.push_back(v[cv]);

        v.erase(v.begin() + static_cast<std::vector<std::uint32_t>::difference_type>(w));
        nv--;
        count = 2 * nv;
        break;
      }
    }
  }

  return indices;
}

}  // namespace strada::tess
