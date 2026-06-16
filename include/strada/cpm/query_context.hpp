#pragma once

#include <optional>
#include <strada/cpm/ids.hpp>

namespace strada::cpm {

struct QueryContext {
  std::optional<RoadId> last_road;
  std::optional<uint32_t> last_segment_idx;
};

}  // namespace strada::cpm
