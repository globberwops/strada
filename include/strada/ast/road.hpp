#pragma once

#include <cstdint>
#include <strada/ast/geometry.hpp>
#include <strada/ast/lanes.hpp>
#include <strada/ast/profiles.hpp>
#include <string>
#include <vector>

namespace strada::ast {

enum class TrafficRule : std::uint8_t { kRht, kLht };

struct Road {
  std::string id;
  double length{};
  std::string junction = "-1";
  TrafficRule rule = TrafficRule::kRht;
  std::string name;
  std::vector<GeometryRecord> plan_view;
  ElevationProfile elevation_profile;
  LateralProfile lateral_profile;
  Lanes lanes;
};

}  // namespace strada::ast
