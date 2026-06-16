#pragma once

#include <strada/cpm/ids.hpp>

namespace strada::cpm {

struct InertialPose {
  double x{};
  double y{};
  double z{};
  double heading{};
  double pitch{};
  double roll{};
};

struct RoadPose {
  double s{};
  double t{};
  double h{};
  double heading{};
  double pitch{};
  double roll{};
  RoadId road{};
};

struct LanePose {
  double s{};
  double t{};
  double h{};
  double heading{};
  double pitch{};
  double roll{};
  RoadId road{};
  LaneId lane{};
};

struct InertialPosition {
  double x{};
  double y{};
  double z{};
};

}  // namespace strada::cpm
