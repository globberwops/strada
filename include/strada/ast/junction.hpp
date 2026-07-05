// SPDX-License-Identifier: BSL-1.0

#pragma once

#include <cstdint>
#include <optional>
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

/// The type of a junction boundary segment.
enum class JunctionSegmentType : std::uint8_t {
  kLane = 0,  ///< The segment runs along a lane boundary.
  kJoint      ///< The segment crosses lanes perpendicular to the road.
};

/// Represents a segment of a junction boundary.
struct JunctionBoundarySegment {
  JunctionSegmentType type{JunctionSegmentType::kLane};  ///< Type of the segment (lane or joint).
  std::string road_id;                                   ///< ID of the road used for the segment.

  // Attributes for type="lane"
  std::optional<int> boundary_lane;  ///< ID of the lane (required for type="lane").
  double s_start{};                  ///< Start of the segment (required for type="lane").
  double s_end{};                    ///< End of the segment (required for type="lane").

  // Attributes for type="joint"
  ContactPoint contact_point{ContactPoint::kStart};  ///< Contact point on the road (required for type="joint").
  std::optional<int> joint_lane_start;               ///< Starting lane crossed by the joint segment (optional).
  std::optional<int> joint_lane_end;                 ///< Ending lane crossed by the joint segment (optional).
  double transition_length{};                        ///< Interpolation zone length (optional).

  Extensions extensions;  ///< Non-schema and custom user data extensions.
};

/// Represents the boundary enclosing a junction's traffic area.
struct JunctionBoundary {
  std::vector<JunctionBoundarySegment> segments;  ///< Closed boundary segments in counter-clockwise order.
  Extensions extensions;                          ///< Non-schema and custom user data extensions.
};

/// The type of a junction.
enum class JunctionType : std::uint8_t {
  kCommon = 0,  ///< Maps to XML "default" or used when @type is omitted/absent.
  kCrossing,    ///< Maps to XML "crossing".
  kDirect,      ///< Maps to XML "direct".
  kVirtual      ///< Maps to XML "virtual"
};

/// Represents an ASAM OpenDRIVE junction containing overlapping connections.
struct Junction {
  std::string id;                            ///< Unique ID of the junction.
  std::optional<std::string> name;           ///< Optional human-readable name of the junction.
  JunctionType type{JunctionType::kCommon};  ///< Type of junction (e.g. default, crossing, direct, virtual).
  std::vector<Connection> connections;       ///< Road-to-road connections within the junction.
  std::optional<JunctionBoundary> boundary;  ///< Optional boundary enclosing the junction area.
  Extensions extensions;                     ///< Non-schema and custom user data extensions.
};

}  // namespace strada::ast
