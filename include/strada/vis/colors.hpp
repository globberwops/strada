#pragma once

#include <strada/ast/road.hpp>

namespace strada::vis {

/// Curated premium color palette matching dark-mode aesthetics.
struct Color {
  float r{};
  float g{};
  float b{};
};

/// Curated premium color palette with alpha channel support.
struct ColorA {
  float r{};
  float g{};
  float b{};
  float a{1.0F};
};

// Rosé Pine (Dark) palette colors
constexpr Color kLaneDrivingLeft{.r = 235.0F / 255.0F, .g = 188.0F / 255.0F, .b = 186.0F / 255.0F};
constexpr Color kLaneDrivingRight{.r = 156.0F / 255.0F, .g = 207.0F / 255.0F, .b = 216.0F / 255.0F};
constexpr Color kLaneBiking{.r = 235.0F / 255.0F, .g = 111.0F / 255.0F, .b = 146.0F / 255.0F};
constexpr Color kLaneBorder{.r = 110.0F / 255.0F, .g = 106.0F / 255.0F, .b = 134.0F / 255.0F};
constexpr Color kLaneConnectingRamp{.r = 196.0F / 255.0F, .g = 167.0F / 255.0F, .b = 231.0F / 255.0F};
constexpr Color kLaneCurb{.r = 38.0F / 255.0F, .g = 35.0F / 255.0F, .b = 58.0F / 255.0F};
constexpr Color kLaneEntry{.r = 49.0F / 255.0F, .g = 116.0F / 255.0F, .b = 143.0F / 255.0F};
constexpr Color kLaneExit{.r = 156.0F / 255.0F, .g = 207.0F / 255.0F, .b = 216.0F / 255.0F};
constexpr Color kLaneMedian{.r = 64.0F / 255.0F, .g = 61.0F / 255.0F, .b = 82.0F / 255.0F};
constexpr Color kLaneNone{.r = 25.0F / 255.0F, .g = 23.0F / 255.0F, .b = 36.0F / 255.0F};
constexpr Color kLaneOffRamp{.r = 49.0F / 255.0F, .g = 116.0F / 255.0F, .b = 143.0F / 255.0F};
constexpr Color kLaneOnRamp{.r = 196.0F / 255.0F, .g = 167.0F / 255.0F, .b = 231.0F / 255.0F};
constexpr Color kLaneParking{.r = 42.0F / 255.0F, .g = 40.0F / 255.0F, .b = 55.0F / 255.0F};
constexpr Color kLaneRail{.r = 196.0F / 255.0F, .g = 167.0F / 255.0F, .b = 231.0F / 255.0F};
constexpr Color kLaneRestricted{.r = 235.0F / 255.0F, .g = 111.0F / 255.0F, .b = 146.0F / 255.0F};
constexpr Color kLaneShoulder{.r = 144.0F / 255.0F, .g = 140.0F / 255.0F, .b = 170.0F / 255.0F};
constexpr Color kLaneSidewalk{.r = 110.0F / 255.0F, .g = 106.0F / 255.0F, .b = 134.0F / 255.0F};
constexpr Color kLaneSlipLane{.r = 156.0F / 255.0F, .g = 207.0F / 255.0F, .b = 216.0F / 255.0F};
constexpr Color kLaneStop{.r = 235.0F / 255.0F, .g = 111.0F / 255.0F, .b = 146.0F / 255.0F};
constexpr Color kLaneTram{.r = 196.0F / 255.0F, .g = 167.0F / 255.0F, .b = 231.0F / 255.0F};

// Feature specific colors
constexpr Color kReferenceLineColor{.r = 235.0F / 255.0F, .g = 111.0F / 255.0F, .b = 146.0F / 255.0F};
constexpr Color kJunctionBoundaryColor{.r = 246.0F / 255.0F, .g = 193.0F / 255.0F, .b = 119.0F / 255.0F};
constexpr Color kObjectColor{.r = 235.0F / 255.0F, .g = 188.0F / 255.0F, .b = 186.0F / 255.0F};
constexpr Color kSignalColor{.r = 156.0F / 255.0F, .g = 207.0F / 255.0F, .b = 216.0F / 255.0F};

// UI Glassmorphic panel styling colors
constexpr ColorA kUIBorder{.r = 110.0F / 255.0F, .g = 106.0F / 255.0F, .b = 134.0F / 255.0F, .a = 1.0F};
constexpr ColorA kUIBackground{.r = 33.0F / 255.0F, .g = 32.0F / 255.0F, .b = 48.0F / 255.0F, .a = 220.0F / 255.0F};
constexpr Color kUIBackgroundOpaque{.r = 33.0F / 255.0F, .g = 32.0F / 255.0F, .b = 48.0F / 255.0F};

// UI Text and Label colors
constexpr Color kTextGold{.r = 246.0F / 255.0F, .g = 193.0F / 255.0F, .b = 119.0F / 255.0F};
constexpr Color kTextAmber{.r = 235.0F / 255.0F, .g = 188.0F / 255.0F, .b = 186.0F / 255.0F};
constexpr Color kTextLabel{.r = 144.0F / 255.0F, .g = 140.0F / 255.0F, .b = 170.0F / 255.0F};
constexpr Color kTextValue{.r = 156.0F / 255.0F, .g = 207.0F / 255.0F, .b = 216.0F / 255.0F};
constexpr Color kTextDescription{.r = 110.0F / 255.0F, .g = 106.0F / 255.0F, .b = 134.0F / 255.0F};
constexpr Color kTextLight{.r = 224.0F / 255.0F, .g = 222.0F / 255.0F, .b = 244.0F / 255.0F};

// Compass and scale bar specific colors
constexpr ColorA kCompassEast{.r = 156.0F / 255.0F, .g = 207.0F / 255.0F, .b = 216.0F / 255.0F, .a = 1.0F};
constexpr ColorA kCompassNorth{.r = 235.0F / 255.0F, .g = 111.0F / 255.0F, .b = 146.0F / 255.0F, .a = 1.0F};

// 3D Scene overlays
constexpr ColorA kHoverHighlight{.r = 235.0F / 255.0F, .g = 111.0F / 255.0F, .b = 146.0F / 255.0F, .a = 0.4F};
constexpr ColorA kJunctionHighlight{.r = 246.0F / 255.0F, .g = 193.0F / 255.0F, .b = 119.0F / 255.0F, .a = 0.12F};
constexpr ColorA kRouteHighlight{.r = 156.0F / 255.0F, .g = 207.0F / 255.0F, .b = 216.0F / 255.0F, .a = 0.4F};
constexpr Color kRouteErrorText{.r = 235.0F / 255.0F, .g = 111.0F / 255.0F, .b = 146.0F / 255.0F};
constexpr ColorA kRouteErrorBorder{.r = 235.0F / 255.0F, .g = 111.0F / 255.0F, .b = 146.0F / 255.0F, .a = 1.0F};
constexpr ColorA kRouteErrorBackground{
    .r = 235.0F / 255.0F, .g = 111.0F / 255.0F, .b = 146.0F / 255.0F, .a = 30.0F / 255.0F};
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
