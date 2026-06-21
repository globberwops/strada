// SPDX-License-Identifier: BSL-1.0

#pragma once

#include <cstdint>
#include <strada/ast/extensions.hpp>
#include <strada/ast/geometry.hpp>
#include <strada/ast/lanes.hpp>
#include <strada/ast/profiles.hpp>
#include <string>
#include <vector>

namespace strada::ast {

/// Traffic rule of the road (Right Hand Traffic or Left Hand Traffic).
enum class TrafficRule : std::uint8_t {
  kRht = 0,  ///< Right-hand traffic.
  kLht       ///< Left-hand traffic.
};

/// Represents an individual road inside the map network.
struct Road {
  std::string id;                         ///< Unique ID of the road.
  double length{};                        ///< Total length of the road reference line.
  std::string junction{"-1"};             ///< ID of the junction this road belongs to (-1 for none).
  TrafficRule rule{TrafficRule::kRht};    ///< Traffic rule (RHT/LHT).
  std::string name;                       ///< Optional human-readable name of the road.
  std::vector<GeometryRecord> plan_view;  ///< The plan-view geometry segments of the reference line.
  ElevationProfile elevation_profile;     ///< Vertical elevation profile.
  LateralProfile lateral_profile;         ///< Superelevation and lateral shapes.
  Lanes lanes;                            ///< Lanes structure (width, offset, section groups).
  Extensions extensions;                  ///< Non-schema and custom user data extensions.
};

}  // namespace strada::ast
