// SPDX-License-Identifier: BSL-1.0

#pragma once

#include <optional>
#include <strada/cpm/ids.hpp>

namespace strada::cpm {

/// Mutable query context thread-owned cache to exploit spatial and temporal coherence.
///
/// This structure holds state from the last query (like the active road and BVH segment index),
/// enabling the hot-path snapping algorithms to fast-path subsequent nearby queries.
struct QueryContext {
  std::optional<RoadId> last_road;                ///< Cache of the last queried RoadId.
  std::optional<std::uint32_t> last_segment_idx;  ///< Cache of the last matched reference line segment index.
};

}  // namespace strada::cpm
