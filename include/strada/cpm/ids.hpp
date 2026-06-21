// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <cstdint>

namespace strada::cpm {

/// Strongly-typed identifier for road segments in the compiled physics model.
enum class RoadId : std::uint32_t {};

/// Strongly-typed identifier for individual lanes in the compiled physics model.
enum class LaneId : std::uint32_t {};

}  // namespace strada::cpm
