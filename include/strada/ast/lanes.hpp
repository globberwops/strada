// SPDX-License-Identifier: BSL-1.0

#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace strada::ast {

/// Type of lane defined by ASAM OpenDRIVE.
enum class LaneType : std::uint8_t {
  kHov = 0,
  kBidirectional,
  kBiking,
  kBorder,
  kBus,
  kConnectingRamp,
  kCurb,
  kDriving,
  kEntry,
  kExit,
  kMedian,
  kMwyEntry,
  kMwyExit,
  kNone,
  kOffRamp,
  kOnRamp,
  kParking,
  kRail,
  kRestricted,
  kRoadWorks,
  kShared,
  kShoulder,
  kSidewalk,
  kSlipLane,
  kSpecial1,
  kSpecial2,
  kSpecial3,
  kStop,
  kTaxi,
  kTram,
  kWalking
};

/// Converts a LaneType enum value to its corresponding OpenDRIVE string representation.
inline auto ToString(LaneType type) noexcept -> std::string_view {
  switch (type) {
    case LaneType::kHov:
      return "hov";
    case LaneType::kBidirectional:
      return "bidirectional";
    case LaneType::kBiking:
      return "biking";
    case LaneType::kBorder:
      return "border";
    case LaneType::kBus:
      return "bus";
    case LaneType::kConnectingRamp:
      return "connectingRamp";
    case LaneType::kCurb:
      return "curb";
    case LaneType::kDriving:
      return "driving";
    case LaneType::kEntry:
      return "entry";
    case LaneType::kExit:
      return "exit";
    case LaneType::kMedian:
      return "median";
    case LaneType::kMwyEntry:
      return "mwyEntry";
    case LaneType::kMwyExit:
      return "mwyExit";
    case LaneType::kNone:
      return "none";
    case LaneType::kOffRamp:
      return "offRamp";
    case LaneType::kOnRamp:
      return "onRamp";
    case LaneType::kParking:
      return "parking";
    case LaneType::kRail:
      return "rail";
    case LaneType::kRestricted:
      return "restricted";
    case LaneType::kRoadWorks:
      return "roadWorks";
    case LaneType::kShared:
      return "shared";
    case LaneType::kShoulder:
      return "shoulder";
    case LaneType::kSidewalk:
      return "sidewalk";
    case LaneType::kSlipLane:
      return "slipLane";
    case LaneType::kSpecial1:
      return "special1";
    case LaneType::kSpecial2:
      return "special2";
    case LaneType::kSpecial3:
      return "special3";
    case LaneType::kStop:
      return "stop";
    case LaneType::kTaxi:
      return "taxi";
    case LaneType::kTram:
      return "tram";
    case LaneType::kWalking:
      return "walking";
  }
  return "";
}

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
  LaneType type{LaneType::kNone};   ///< Type of lane (e.g. driving, sidewalk, shoulder).
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
