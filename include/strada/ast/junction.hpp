// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <cstdint>
#include <strada/ast/extensions.hpp>
#include <string>
#include <vector>

namespace strada::ast {

/// The contact point of a connecting road at a junction.
enum class ContactPoint : std::uint8_t {
  kStart = 0,  ///< The link connects to the start s-station of the connecting road.
  kEnd         ///< The link connects to the end s-station of the connecting road.
};

/// Represents lane-level connectivity inside a junction connection.
struct LaneLink {
  int from{};  ///< Source lane integer ID in the incoming road.
  int to{};    ///< Target lane integer ID in the connecting road.
};

/// Describes connection details between roads inside a junction.
struct Connection {
  std::string id;                                    ///< Unique ID of the connection.
  std::string incoming_road;                         ///< ID of the road leading into the junction.
  std::string connecting_road;                       ///< ID of the road acting as a path inside the junction.
  ContactPoint contact_point{ContactPoint::kStart};  ///< Contact direction of the connecting road.
  std::vector<LaneLink> lane_links;                  ///< Direct lane connectivity maps.
};

/// Represents an ASAM OpenDRIVE junction containing overlapping connections.
struct Junction {
  std::string id;                       ///< Unique ID of the junction.
  std::string name;                     ///< Optional human-readable name of the junction.
  std::string type;                     ///< Type of junction (e.g. "default", "direct").
  std::vector<Connection> connections;  ///< Road-to-road connections within the junction.
  Extensions extensions;                ///< Non-schema and custom user data extensions.
};

}  // namespace strada::ast
