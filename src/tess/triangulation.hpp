// SPDX-License-Identifier: BSL-1.0

#pragma once

#include <cstdint>
#include <strada/tess/tessellator.hpp>
#include <vector>

namespace strada::tess {

/// Triangulates a closed polygon loop using a CPU-side ear-clipping algorithm.
/// @param vertices Closed loop vertices.
/// @return Triangulation indices.
[[nodiscard]] auto TriangulatePolygon(const std::vector<Vertex>& vertices) -> std::vector<std::uint32_t>;

}  // namespace strada::tess
