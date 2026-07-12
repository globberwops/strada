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

// Premium dark-mode palette colors
constexpr Color kLaneDrivingLeft{.r = 239.0F / 255.0F, .g = 215.0F / 255.0F, .b = 171.0F / 255.0F};
constexpr Color kLaneDrivingRight{.r = 205.0F / 255.0F, .g = 216.0F / 255.0F, .b = 232.0F / 255.0F};
constexpr Color kLaneBiking{.r = 207.0F / 255.0F, .g = 16.0F / 255.0F, .b = 45.0F / 255.0F};
constexpr Color kLaneBorder{.r = 165.0F / 255.0F, .g = 94.0F / 255.0F, .b = 55.0F / 255.0F};
constexpr Color kLaneConnectingRamp{.r = 168.0F / 255.0F, .g = 211.0F / 255.0F, .b = 0.0F / 255.0F};
constexpr Color kLaneCurb{.r = 151.0F / 255.0F, .g = 120.0F / 255.0F, .b = 211.0F / 255.0F};
constexpr Color kLaneEntry{.r = 234.0F / 255.0F, .g = 217.0F / 255.0F, .b = 96.0F / 255.0F};
constexpr Color kLaneExit{.r = 103.0F / 255.0F, .g = 153.0F / 255.0F, .b = 204.0F / 255.0F};
constexpr Color kLaneMedian{.r = 124.0F / 255.0F, .g = 84.0F / 255.0F, .b = 71.0F / 255.0F};
constexpr Color kLaneNone{.r = 147.0F / 255.0F, .g = 149.0F / 255.0F, .b = 152.0F / 255.0F};
constexpr Color kLaneOffRamp{.r = 35.0F / 255.0F, .g = 121.0F / 255.0F, .b = 185.0F / 255.0F};
constexpr Color kLaneOnRamp{.r = 255.0F / 255.0F, .g = 212.0F / 255.0F, .b = 2.0F / 255.0F};
constexpr Color kLaneParking{.r = 98.0F / 255.0F, .g = 38.0F / 255.0F, .b = 158.0F / 255.0F};
constexpr Color kLaneRail{.r = 56.0F / 255.0F, .g = 43.0F / 255.0F, .b = 178.0F / 255.0F};
constexpr Color kLaneRestricted{.r = 255.0F / 255.0F, .g = 103.0F / 255.0F, .b = 27.0F / 255.0F};
constexpr Color kLaneShoulder{.r = 0.0F / 255.0F, .g = 98.0F / 255.0F, .b = 65.0F / 255.0F};
constexpr Color kLaneSidewalk{.r = 121.0F / 255.0F, .g = 36.0F / 255.0F, .b = 47.0F / 255.0F};
constexpr Color kLaneSlipLane{.r = 0.0F / 255.0F, .g = 148.0F / 255.0F, .b = 94.0F / 255.0F};
constexpr Color kLaneStop{.r = 146.0F / 255.0F, .g = 213.0F / 255.0F, .b = 172.0F / 255.0F};
constexpr Color kLaneTram{.r = 109.0F / 255.0F, .g = 109.0F / 255.0F, .b = 226.0F / 255.0F};

// Feature specific colors
constexpr Color kReferenceLineColor{.r = 1.0F, .g = 0.0F, .b = 0.0F};
constexpr Color kJunctionBoundaryColor{.r = 245.0F / 255.0F, .g = 197.0F / 255.0F, .b = 61.0F / 255.0F};
constexpr Color kObjectColor{.r = 1.0F, .g = 145.0F / 255.0F, .b = 0.0F};
constexpr Color kSignalColor{.r = 0.0F, .g = 229.0F / 255.0F, .b = 1.0F};

// UI Glassmorphic panel styling colors
constexpr ColorA kUIBorder{.r = 45.0F / 255.0F, .g = 51.0F / 255.0F, .b = 64.0F / 255.0F, .a = 1.0F};
constexpr ColorA kUIBackground{.r = 26.0F / 255.0F, .g = 29.0F / 255.0F, .b = 36.0F / 255.0F, .a = 220.0F / 255.0F};
constexpr Color kUIBackgroundOpaque{.r = 26.0F / 255.0F, .g = 29.0F / 255.0F, .b = 36.0F / 255.0F};

// UI Text and Label colors
constexpr Color kTextGold{.r = 255.0F / 255.0F, .g = 204.0F / 255.0F, .b = 0.0F};
constexpr Color kTextAmber{.r = 245.0F / 255.0F, .g = 197.0F / 255.0F, .b = 61.0F / 255.0F};
constexpr Color kTextLabel{.r = 160.0F / 255.0F, .g = 170.0F / 255.0F, .b = 184.0F / 255.0F};
constexpr Color kTextValue{.r = 100.0F / 255.0F, .g = 181.0F / 255.0F, .b = 246.0F / 255.0F};
constexpr Color kTextDescription{.r = 180.0F / 255.0F, .g = 188.0F / 255.0F, .b = 204.0F / 255.0F};
constexpr Color kTextLight{.r = 240.0F / 255.0F, .g = 240.0F / 255.0F, .b = 240.0F / 255.0F};

// Compass and scale bar specific colors
constexpr ColorA kCompassEast{.r = 100.0F / 255.0F, .g = 181.0F / 255.0F, .b = 246.0F / 255.0F, .a = 1.0F};
constexpr ColorA kCompassNorth{.r = 255.0F / 255.0F, .g = 110.0F / 255.0F, .b = 110.0F / 255.0F, .a = 1.0F};

// 3D Scene overlays
constexpr ColorA kHoverHighlight{.r = 1.0F, .g = 0.0F, .b = 0.0F, .a = 0.4F};
constexpr ColorA kJunctionHighlight{.r = 245.0F / 255.0F, .g = 197.0F / 255.0F, .b = 61.0F / 255.0F, .a = 0.12F};

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
