// SPDX-License-Identifier: BSL-1.0

#pragma once

#include <strada/ast/road.hpp>

namespace strada::vis {

/// Curated premium color palette matching dark-mode aesthetics.
struct Color {
  float r{};
  float g{};
  float b{};
};

// Premium dark-mode palette colors
constexpr Color kLaneDrivingLeft{239.0F / 255.0F, 215.0F / 255.0F, 171.0F / 255.0F};
constexpr Color kLaneDrivingRight{205.0F / 255.0F, 216.0F / 255.0F, 232.0F / 255.0F};
constexpr Color kLaneBiking{207.0F / 255.0F, 16.0F / 255.0F, 45.0F / 255.0F};
constexpr Color kLaneBorder{165.0F / 255.0F, 94.0F / 255.0F, 55.0F / 255.0F};
constexpr Color kLaneConnectingRamp{168.0F / 255.0F, 211.0F / 255.0F, 0.0F / 255.0F};
constexpr Color kLaneCurb{151.0F / 255.0F, 120.0F / 255.0F, 211.0F / 255.0F};
constexpr Color kLaneEntry{234.0F / 255.0F, 217.0F / 255.0F, 96.0F / 255.0F};
constexpr Color kLaneExit{103.0F / 255.0F, 153.0F / 255.0F, 204.0F / 255.0F};
constexpr Color kLaneMedian{124.0F / 255.0F, 84.0F / 255.0F, 71.0F / 255.0F};
constexpr Color kLaneNone{147.0F / 255.0F, 149.0F / 255.0F, 152.0F / 255.0F};
constexpr Color kLaneOffRamp{35.0F / 255.0F, 121.0F / 255.0F, 185.0F / 255.0F};
constexpr Color kLaneOnRamp{255.0F / 255.0F, 212.0F / 255.0F, 2.0F / 255.0F};
constexpr Color kLaneParking{98.0F / 255.0F, 38.0F / 255.0F, 158.0F / 255.0F};
constexpr Color kLaneRail{56.0F / 255.0F, 43.0F / 255.0F, 178.0F / 255.0F};
constexpr Color kLaneRestricted{255.0F / 255.0F, 103.0F / 255.0F, 27.0F / 255.0F};
constexpr Color kLaneShoulder{0.0F / 255.0F, 98.0F / 255.0F, 65.0F / 255.0F};
constexpr Color kLaneSidewalk{121.0F / 255.0F, 36.0F / 255.0F, 47.0F / 255.0F};
constexpr Color kLaneSlipLane{0.0F / 255.0F, 148.0F / 255.0F, 94.0F / 255.0F};
constexpr Color kLaneStop{146.0F / 255.0F, 213.0F / 255.0F, 172.0F / 255.0F};
constexpr Color kLaneTram{109.0F / 255.0F, 109.0F / 255.0F, 226.0F / 255.0F};

// Feature specific colors
constexpr Color kReferenceLineColor{1.0F, 0.0F, 0.0F};
constexpr Color kJunctionBoundaryColor{245.0F / 255.0F, 197.0F / 255.0F, 61.0F / 255.0F};
constexpr Color kObjectColor{1.0F, 145.0F / 255.0F, 0.0F};
constexpr Color kSignalColor{0.0F, 229.0F / 255.0F, 1.0F};

/// Color lookup helper based on lane type and original lane ID.
constexpr auto GetLaneColor(ast::LaneType lane_type, int original_lane_id) noexcept -> Color {
  switch (lane_type) {
    case ast::LaneType::kHov:
    case ast::LaneType::kBidirectional:
    case ast::LaneType::kBus:
    case ast::LaneType::kTaxi:
    case ast::LaneType::kRoadWorks:
    case ast::LaneType::kShared:
    case ast::LaneType::kDriving: {
      if (original_lane_id < 0) {
        return kLaneDrivingLeft;
      }
      return kLaneDrivingRight;
    }
    case ast::LaneType::kBiking:
      return kLaneBiking;
    case ast::LaneType::kBorder:
      return kLaneBorder;
    case ast::LaneType::kConnectingRamp:
      return kLaneConnectingRamp;
    case ast::LaneType::kCurb:
      return kLaneCurb;
    case ast::LaneType::kMwyEntry:
    case ast::LaneType::kEntry:
      return kLaneEntry;
    case ast::LaneType::kMwyExit:
    case ast::LaneType::kExit:
      return kLaneExit;
    case ast::LaneType::kMedian:
      return kLaneMedian;
    case ast::LaneType::kSpecial1:
    case ast::LaneType::kSpecial2:
    case ast::LaneType::kSpecial3:
    case ast::LaneType::kNone:
      return kLaneNone;
    case ast::LaneType::kOffRamp:
      return kLaneOffRamp;
    case ast::LaneType::kOnRamp:
      return kLaneOnRamp;
    case ast::LaneType::kParking:
      return kLaneParking;
    case ast::LaneType::kRail:
      return kLaneRail;
    case ast::LaneType::kRestricted:
      return kLaneRestricted;
    case ast::LaneType::kShoulder:
      return kLaneShoulder;
    case ast::LaneType::kSidewalk:
    case ast::LaneType::kWalking:
      return kLaneSidewalk;
    case ast::LaneType::kSlipLane:
      return kLaneSlipLane;
    case ast::LaneType::kStop:
      return kLaneStop;
    case ast::LaneType::kTram:
      return kLaneTram;
  }
  return kLaneNone;
}

}  // namespace strada::vis
