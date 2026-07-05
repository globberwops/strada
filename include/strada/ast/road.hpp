// SPDX-License-Identifier: BSL-1.0

#pragma once

#include <cstdint>
#include <optional>
#include <strada/ast/extensions.hpp>
#include <strada/ast/geometry.hpp>
#include <strada/ast/lanes.hpp>
#include <strada/ast/objects.hpp>
#include <strada/ast/profiles.hpp>
#include <string>
#include <vector>

namespace strada::ast {

/// Traffic rule of the road (Right Hand Traffic or Left Hand Traffic).
enum class TrafficRule : std::uint8_t {
  kRht = 0,  ///< Right-hand traffic.
  kLht       ///< Left-hand traffic.
};

/// Scoped enum representing OpenDRIVE road types.
enum class RoadType : std::uint8_t {
  kUnknown = 0,
  kBicycle,
  kLowSpeed,
  kPedestrian,
  kMotorway,
  kRural,
  kTownArterial,
  kTownCollector,
  kTownExpressway,
  kTownLocal,
  kTownPlayStreet,
  kTownPrivate,
  kTown
};

/// Represents a road type entry at a specific s-station.
struct RoadTypeRecord {
  double s{};                         ///< Start s-station of the road type entry.
  RoadType type{RoadType::kUnknown};  ///< Type of the road.
};

/// Represents an individual road inside the map network.
struct Road {
  std::string id;                         ///< Unique ID of the road.
  double length{};                        ///< Total length of the road reference line.
  std::string junction{"-1"};             ///< ID of the junction this road belongs to (-1 for none).
  TrafficRule rule{TrafficRule::kRht};    ///< Traffic rule (RHT/LHT).
  std::optional<std::string> name;        ///< Optional human-readable name of the road.
  std::vector<GeometryRecord> plan_view;  ///< The plan-view geometry segments of the reference line.
  ElevationProfile elevation_profile;     ///< Vertical elevation profile.
  LateralProfile lateral_profile;         ///< Superelevation and lateral shapes.
  Lanes lanes;                            ///< Lanes structure (width, offset, section groups).
  std::vector<RoadTypeRecord> types;      ///< Road type records along the road.
  std::vector<Bridge> bridges;            ///< Bridges along the road.
  std::vector<Tunnel> tunnels;            ///< Tunnels along the road.
  Extensions extensions;                  ///< Non-schema and custom user data extensions.
};

}  // namespace strada::ast
