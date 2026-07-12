#include "triangulation.hpp"

namespace strada::tess {

auto TriangulatePolygon(const std::vector<Vertex>& vertices) -> std::vector<std::uint32_t> {
  std::vector<std::uint32_t> indices;
  if (vertices.size() < 3) {
    return indices;
  }

  const std::size_t n = vertices.size();
  std::vector<std::uint32_t> v(n);

  // Compute signed area to determine winding order
  double area = 0.0;
  for (std::size_t i = 0; i < n; ++i) {
    const auto& p1 = vertices[i];
    const auto& p2 = vertices[(i + 1) % n];
    area += (static_cast<double>(p1.x) * static_cast<double>(p2.y)) -
            (static_cast<double>(p2.x) * static_cast<double>(p1.y));
  }

  // Winding order: if area is negative, we want CW, if positive CCW.
  if (area < 0.0) {
    for (std::size_t i = 0; i < n; ++i) {
      v[i] = static_cast<std::uint32_t>(n - 1 - i);
    }
  } else {
    for (std::size_t i = 0; i < n; ++i) {
      v[i] = static_cast<std::uint32_t>(i);
    }
  }

  // Helper functions inside TriangulatePolygon
  auto inside_triangle = [](float ax, float ay, float bx, float by, float cx, float cy, float px, float py) -> bool {
    const float ax_px = ax - px;
    const float ay_py = ay - py;
    const float bx_px = bx - px;
    const float by_py = by - py;
    const float cx_px = cx - px;
    const float cy_py = cy - py;

    const float ccw_ab = (ax_px * by_py) - (ay_py * bx_px);
    const float ccw_bc = (bx_px * cy_py) - (by_py * cx_px);
    const float ccw_ca = (cx_px * ay_py) - (cy_py * ax_px);

    return (ccw_ab >= 0.0F && ccw_bc >= 0.0F && ccw_ca >= 0.0F) || (ccw_ab <= 0.0F && ccw_bc <= 0.0F && ccw_ca <= 0.0F);
  };

  auto is_ear = [&](std::size_t u, std::size_t w, std::size_t cv, const std::vector<std::uint32_t>& v_indices) -> bool {
    const auto& a = vertices[v_indices[u]];
    const auto& b = vertices[v_indices[w]];
    const auto& c = vertices[v_indices[cv]];

    // Check if triangle is convex (CCW)
    const float cross_product = ((b.x - a.x) * (c.y - b.y)) - ((b.y - a.y) * (c.x - b.x));
    if (cross_product <= 0.0F) {
      return false;
    }

    // Check if any other vertex is inside the triangle
    for (std::size_t p = 0; p < v_indices.size(); ++p) {
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

  std::size_t count = 2 * n;  // Prevent infinite loop on degenerate polygons
  std::size_t nv = n;
  while (nv > 2) {
    if (count == 0) {
      // Degenerate fallback to triangle fan to avoid hanging
      for (std::size_t i = 1; i < nv - 1; ++i) {
        indices.push_back(v[0]);
        indices.push_back(v[i]);
        indices.push_back(v[i + 1]);
      }
      return indices;
    }
    count--;

    for (std::size_t i = 0; i < nv; ++i) {
      const std::size_t u = (i == 0) ? (nv - 1) : (i - 1);
      const std::size_t w = i;
      const std::size_t cv = (i + 1 == nv) ? 0 : (i + 1);

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
