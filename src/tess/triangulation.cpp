// SPDX-License-Identifier: BSL-1.0

#include "triangulation.hpp"

namespace strada::tess {

auto TriangulatePolygon(const std::vector<Vertex>& vertices) -> std::vector<std::uint32_t> {
  std::vector<std::uint32_t> indices;
  if (vertices.size() < 3) {
    return indices;
  }

  const std::size_t kN = vertices.size();
  std::vector<std::uint32_t> v(kN);

  // Compute signed area to determine winding order
  double area = 0.0;
  for (std::size_t i = 0; i < kN; ++i) {
    const auto& p1 = vertices[i];
    const auto& p2 = vertices[(i + 1) % kN];
    area += (static_cast<double>(p1.x) * p2.y) - (static_cast<double>(p2.x) * p1.y);
  }

  // Winding order: if area is negative, we want CW, if positive CCW.
  if (area < 0.0) {
    for (std::size_t i = 0; i < kN; ++i) {
      v[i] = static_cast<std::uint32_t>(kN - 1 - i);
    }
  } else {
    for (std::size_t i = 0; i < kN; ++i) {
      v[i] = static_cast<std::uint32_t>(i);
    }
  }

  // Helper functions inside TriangulatePolygon
  auto inside_triangle = [](float ax, float ay, float bx, float by, float cx, float cy, float px, float py) -> bool {
    const float kAxPx = ax - px;
    const float kAyPy = ay - py;
    const float kBxPx = bx - px;
    const float kByPy = by - py;
    const float kCxPx = cx - px;
    const float kCyPy = cy - py;

    const float kCcwAb = (kAxPx * kByPy) - (kAyPy * kBxPx);
    const float kCcwBc = (kBxPx * kCyPy) - (kByPy * kCxPx);
    const float kCcwCa = (kCxPx * kAyPy) - (kCyPy * kAxPx);

    return (kCcwAb >= 0.0F && kCcwBc >= 0.0F && kCcwCa >= 0.0F) || (kCcwAb <= 0.0F && kCcwBc <= 0.0F && kCcwCa <= 0.0F);
  };

  auto is_ear = [&](std::size_t u, std::size_t w, std::size_t cv, const std::vector<std::uint32_t>& v_indices) -> bool {
    const auto& a = vertices[v_indices[u]];
    const auto& b = vertices[v_indices[w]];
    const auto& c = vertices[v_indices[cv]];

    // Check if triangle is convex (CCW)
    const float kCrossProduct = ((b.x - a.x) * (c.y - b.y)) - ((b.y - a.y) * (c.x - b.x));
    if (kCrossProduct <= 0.0F) {
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

  std::size_t count = 2 * kN;  // Prevent infinite loop on degenerate polygons
  std::size_t nv = kN;
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
      const std::size_t kU = (i == 0) ? (nv - 1) : (i - 1);
      const std::size_t kW = i;
      const std::size_t kCv = (i + 1 == nv) ? 0 : (i + 1);

      if (is_ear(kU, kW, kCv, v)) {
        indices.push_back(v[kU]);
        indices.push_back(v[kW]);
        indices.push_back(v[kCv]);

        v.erase(v.begin() + static_cast<std::vector<std::uint32_t>::difference_type>(kW));
        nv--;
        count = 2 * nv;
        break;
      }
    }
  }

  return indices;
}

}  // namespace strada::tess
