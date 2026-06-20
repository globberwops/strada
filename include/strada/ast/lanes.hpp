// Copyright 2026 Google LLC
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <optional>
#include <string>
#include <vector>

namespace strada::ast {

/// Defines the lateral offset of the road reference line from the center lane.
struct LaneOffset {
  double s{};  ///< Start s-station of the offset entry.
  double a{};  ///< Constant coefficient (a).
  double b{};  ///< Linear coefficient (b).
  double c{};  ///< Quadratic coefficient (c).
  double d{};  ///< Cubic coefficient (d).
};

/// Defines the width of a lane within a specific lane section.
struct LaneWidth {
  double s_offset{};  ///< Start station of the width entry, relative to the lane section start.
  double a{};         ///< Constant coefficient (a).
  double b{};         ///< Linear coefficient (b).
  double c{};         ///< Quadratic coefficient (c).
  double d{};         ///< Cubic coefficient (d).
};

/// Defines the height offset of a lane at its inner and outer boundaries.
struct LaneHeight {
  double s_offset{};  ///< Start station of the height entry, relative to the lane section start.
  double inner{};     ///< Height offset at the inner boundary (closer to reference line).
  double outer{};     ///< Height offset at the outer boundary (further from reference line).
};

/// Represents a single lane segment in a lane section.
struct Lane {
  int id{};                         ///< Unique integer ID of the lane (left > 0, right < 0, center = 0).
  std::string type;                 ///< Type of lane (e.g. "driving", "sidewalk", "shoulder").
  bool level{};                     ///< Flag indicating if the lane ignores road superelevation.
  std::optional<int> predecessor;   ///< ID of the predecessor lane in the previous section.
  std::optional<int> successor;     ///< ID of the successor lane in the next section.
  std::vector<LaneWidth> widths;    ///< Polynomial width segments for this lane.
  std::vector<LaneHeight> heights;  ///< Height offset segments for this lane.
};

/// Grouping of lanes at a given s-station along the road.
struct LaneSection {
  double s{};                ///< Start s-station of the lane section.
  std::vector<Lane> left;    ///< Lanes to the left of the center lane (positive IDs).
  std::vector<Lane> center;  ///< Center lanes (ID = 0).
  std::vector<Lane> right;   ///< Lanes to the right of the center lane (negative IDs).
};

/// Top-level container for lane offsets and lane sections of a road.
struct Lanes {
  std::vector<LaneOffset> offsets;    ///< Road-level reference line lateral offsets.
  std::vector<LaneSection> sections;  ///< Contiguous lane sections forming the road.
};

}  // namespace strada::ast
