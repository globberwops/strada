#pragma once

#include <optional>
#include <strada/cpm/ids.hpp>

namespace strada::cpm {

struct QueryContext {
  std::optional<RoadId> last_road;
};

}  // namespace strada::cpm
