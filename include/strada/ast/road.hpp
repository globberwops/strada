#pragma once

#include <cstdint>
#include <strada/ast/geometry.hpp>
#include <strada/ast/lanes.hpp>
#include <strada/ast/profiles.hpp>
#include <string>
#include <vector>

namespace strada::ast {

enum class TrafficRule : std::uint8_t { RHT, LHT };

struct Road {
  std::string id_;
  double length_{};
  std::string junction_ = "-1";
  TrafficRule rule_ = TrafficRule::RHT;
  std::string name_;
  std::vector<GeometryRecord> plan_view_;
  ElevationProfile elevation_profile_;
  LateralProfile lateral_profile_;
  Lanes lanes_;
};

}  // namespace strada::ast
