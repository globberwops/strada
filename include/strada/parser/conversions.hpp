#pragma once

#include <optional>
#include <strada/ast/geometry.hpp>
#include <strada/ast/junction.hpp>
#include <strada/ast/lanes.hpp>
#include <strada/ast/objects.hpp>
#include <strada/ast/profiles.hpp>
#include <strada/ast/road.hpp>
#include <string_view>

namespace strada::parser {

/// Primary template for converting a string representation to an enum value.
template <typename T>
constexpr auto FromString(std::string_view str) -> std::optional<T>;

// Specialization for ast::TrafficRule
template <>
constexpr auto FromString<ast::TrafficRule>(std::string_view str) -> std::optional<ast::TrafficRule> {
  if (str == "RHT") {
    return ast::TrafficRule::kRht;
  }
  if (str == "LHT") {
    return ast::TrafficRule::kLht;
  }
  return std::nullopt;
}

// Specialization for ast::Orientation
template <>
constexpr auto FromString<ast::Orientation>(std::string_view str) -> std::optional<ast::Orientation> {
  if (str == "none") {
    return ast::Orientation::kNone;
  }
  if (str == "+") {
    return ast::Orientation::kPlus;
  }
  if (str == "-") {
    return ast::Orientation::kMinus;
  }
  return std::nullopt;
}

// Specialization for ast::PRange
template <>
constexpr auto FromString<ast::PRange>(std::string_view str) -> std::optional<ast::PRange> {
  if (str == "normalized") {
    return ast::PRange::kNormalized;
  }
  if (str == "arcLength") {
    return ast::PRange::kArcLength;
  }
  return std::nullopt;
}

// Specialization for ast::ContactPoint
template <>
constexpr auto FromString<ast::ContactPoint>(std::string_view str) -> std::optional<ast::ContactPoint> {
  if (str == "start") {
    return ast::ContactPoint::kStart;
  }
  if (str == "end") {
    return ast::ContactPoint::kEnd;
  }
  return std::nullopt;
}

// Specialization for ast::JunctionSegmentType
template <>
constexpr auto FromString<ast::JunctionSegmentType>(std::string_view str) -> std::optional<ast::JunctionSegmentType> {
  if (str == "lane") {
    return ast::JunctionSegmentType::kLane;
  }
  if (str == "joint") {
    return ast::JunctionSegmentType::kJoint;
  }
  return std::nullopt;
}

// Specialization for ast::StripMode
template <>
constexpr auto FromString<ast::StripMode>(std::string_view str) -> std::optional<ast::StripMode> {
  if (str == "independent") {
    return ast::StripMode::kIndependent;
  }
  if (str == "relative") {
    return ast::StripMode::kRelative;
  }
  return std::nullopt;
}

// Specialization for ast::JunctionType
template <>
constexpr auto FromString<ast::JunctionType>(std::string_view str) -> std::optional<ast::JunctionType> {
  if (str == "default") {
    return ast::JunctionType::kCommon;
  }
  if (str == "crossing") {
    return ast::JunctionType::kCrossing;
  }
  if (str == "direct") {
    return ast::JunctionType::kDirect;
  }
  if (str == "virtual") {
    return ast::JunctionType::kVirtual;
  }
  return std::nullopt;
}

// Specialization for ast::LayerType
template <>
constexpr auto FromString<ast::LayerType>(std::string_view str) -> std::optional<ast::LayerType> {
  if (str == "permanent") {
    return ast::LayerType::kPermanent;
  }
  if (str == "temporary") {
    return ast::LayerType::kTemporary;
  }
  return std::nullopt;
}

// Specialization for ast::TunnelType
template <>
constexpr auto FromString<ast::TunnelType>(std::string_view str) -> std::optional<ast::TunnelType> {
  if (str == "standard") {
    return ast::TunnelType::kStandard;
  }
  if (str == "underpass") {
    return ast::TunnelType::kUnderpass;
  }
  return std::nullopt;
}

// Specialization for ast::BridgeType
template <>
constexpr auto FromString<ast::BridgeType>(std::string_view str) -> std::optional<ast::BridgeType> {
  if (str == "brick") {
    return ast::BridgeType::kBrick;
  }
  if (str == "concrete") {
    return ast::BridgeType::kConcrete;
  }
  if (str == "steel") {
    return ast::BridgeType::kSteel;
  }
  if (str == "wood") {
    return ast::BridgeType::kWood;
  }
  return std::nullopt;
}

// Specialization for ast::LaneType
template <>
constexpr auto FromString<ast::LaneType>(std::string_view str)  // NOLINT(readability-function-cognitive-complexity)
    -> std::optional<ast::LaneType> {
  if (str == "hov") {
    return ast::LaneType::kHov;
  }
  if (str == "bidirectional") {
    return ast::LaneType::kBidirectional;
  }
  if (str == "biking") {
    return ast::LaneType::kBiking;
  }
  if (str == "border") {
    return ast::LaneType::kBorder;
  }
  if (str == "bus") {
    return ast::LaneType::kBus;
  }
  if (str == "connectingRamp") {
    return ast::LaneType::kConnectingRamp;
  }
  if (str == "curb") {
    return ast::LaneType::kCurb;
  }
  if (str == "driving") {
    return ast::LaneType::kDriving;
  }
  if (str == "entry") {
    return ast::LaneType::kEntry;
  }
  if (str == "exit") {
    return ast::LaneType::kExit;
  }
  if (str == "median") {
    return ast::LaneType::kMedian;
  }
  if (str == "mwyEntry") {
    return ast::LaneType::kMwyEntry;
  }
  if (str == "mwyExit") {
    return ast::LaneType::kMwyExit;
  }
  if (str == "none") {
    return ast::LaneType::kNone;
  }
  if (str == "offRamp") {
    return ast::LaneType::kOffRamp;
  }
  if (str == "onRamp") {
    return ast::LaneType::kOnRamp;
  }
  if (str == "parking") {
    return ast::LaneType::kParking;
  }
  if (str == "rail") {
    return ast::LaneType::kRail;
  }
  if (str == "restricted") {
    return ast::LaneType::kRestricted;
  }
  if (str == "roadWorks") {
    return ast::LaneType::kRoadWorks;
  }
  if (str == "shared") {
    return ast::LaneType::kShared;
  }
  if (str == "shoulder") {
    return ast::LaneType::kShoulder;
  }
  if (str == "sidewalk") {
    return ast::LaneType::kSidewalk;
  }
  if (str == "slipLane") {
    return ast::LaneType::kSlipLane;
  }
  if (str == "special1") {
    return ast::LaneType::kSpecial1;
  }
  if (str == "special2") {
    return ast::LaneType::kSpecial2;
  }
  if (str == "special3") {
    return ast::LaneType::kSpecial3;
  }
  if (str == "stop") {
    return ast::LaneType::kStop;
  }
  if (str == "taxi") {
    return ast::LaneType::kTaxi;
  }
  if (str == "tram") {
    return ast::LaneType::kTram;
  }
  if (str == "walking") {
    return ast::LaneType::kWalking;
  }
  return std::nullopt;
}

// Specialization for ast::ObjectType
template <>
constexpr auto FromString<ast::ObjectType>(std::string_view str)  // NOLINT(readability-function-cognitive-complexity)
    -> std::optional<ast::ObjectType> {
  if (str == "none") {
    return ast::ObjectType::kNone;
  }
  if (str == "obstacle") {
    return ast::ObjectType::kObstacle;
  }
  if (str == "car") {
    return ast::ObjectType::kCar;
  }
  if (str == "pole") {
    return ast::ObjectType::kPole;
  }
  if (str == "tree") {
    return ast::ObjectType::kTree;
  }
  if (str == "vegetation") {
    return ast::ObjectType::kVegetation;
  }
  if (str == "barrier") {
    return ast::ObjectType::kBarrier;
  }
  if (str == "building") {
    return ast::ObjectType::kBuilding;
  }
  if (str == "parkingSpace") {
    return ast::ObjectType::kParkingSpace;
  }
  if (str == "patch") {
    return ast::ObjectType::kPatch;
  }
  if (str == "railing") {
    return ast::ObjectType::kRailing;
  }
  if (str == "trafficIsland") {
    return ast::ObjectType::kTrafficIsland;
  }
  if (str == "crosswalk") {
    return ast::ObjectType::kCrosswalk;
  }
  if (str == "streetLamp") {
    return ast::ObjectType::kStreetLamp;
  }
  if (str == "gantry") {
    return ast::ObjectType::kGantry;
  }
  if (str == "soundBarrier") {
    return ast::ObjectType::kSoundBarrier;
  }
  if (str == "van") {
    return ast::ObjectType::kVan;
  }
  if (str == "bus") {
    return ast::ObjectType::kBus;
  }
  if (str == "trailer") {
    return ast::ObjectType::kTrailer;
  }
  if (str == "bike") {
    return ast::ObjectType::kBike;
  }
  if (str == "motorbike") {
    return ast::ObjectType::kMotorbike;
  }
  if (str == "tram") {
    return ast::ObjectType::kTram;
  }
  if (str == "train") {
    return ast::ObjectType::kTrain;
  }
  if (str == "pedestrian") {
    return ast::ObjectType::kPedestrian;
  }
  if (str == "wind") {
    return ast::ObjectType::kWind;
  }
  if (str == "roadMark") {
    return ast::ObjectType::kRoadMark;
  }
  if (str == "roadSurface") {
    return ast::ObjectType::kRoadSurface;
  }
  return std::nullopt;
}

// Specialization for ast::RoadType
template <>
constexpr auto FromString<ast::RoadType>(std::string_view str) -> std::optional<ast::RoadType> {
  if (str == "unknown") {
    return ast::RoadType::kUnknown;
  }
  if (str == "bicycle") {
    return ast::RoadType::kBicycle;
  }
  if (str == "lowSpeed") {
    return ast::RoadType::kLowSpeed;
  }
  if (str == "pedestrian") {
    return ast::RoadType::kPedestrian;
  }
  if (str == "motorway") {
    return ast::RoadType::kMotorway;
  }
  if (str == "rural") {
    return ast::RoadType::kRural;
  }
  if (str == "townArterial") {
    return ast::RoadType::kTownArterial;
  }
  if (str == "townCollector") {
    return ast::RoadType::kTownCollector;
  }
  if (str == "townExpressway") {
    return ast::RoadType::kTownExpressway;
  }
  if (str == "townLocal") {
    return ast::RoadType::kTownLocal;
  }
  if (str == "townPlayStreet") {
    return ast::RoadType::kTownPlayStreet;
  }
  if (str == "townPrivate") {
    return ast::RoadType::kTownPrivate;
  }
  if (str == "town") {
    return ast::RoadType::kTown;
  }
  return std::nullopt;
}

/// Converts ast::TrafficRule to its string representation.
constexpr auto ToString(ast::TrafficRule val) noexcept -> std::string_view {
  switch (val) {
    case ast::TrafficRule::kRht:
      return "RHT";
    case ast::TrafficRule::kLht:
      return "LHT";
  }
  return "";
}

/// Converts ast::Orientation to its string representation.
constexpr auto ToString(ast::Orientation val) noexcept -> std::string_view {
  switch (val) {
    case ast::Orientation::kNone:
      return "none";
    case ast::Orientation::kPlus:
      return "+";
    case ast::Orientation::kMinus:
      return "-";
  }
  return "";
}

/// Converts ast::PRange to its string representation.
constexpr auto ToString(ast::PRange val) noexcept -> std::string_view {
  switch (val) {
    case ast::PRange::kNormalized:
      return "normalized";
    case ast::PRange::kArcLength:
      return "arcLength";
  }
  return "";
}

/// Converts ast::ContactPoint to its string representation.
constexpr auto ToString(ast::ContactPoint val) noexcept -> std::string_view {
  switch (val) {
    case ast::ContactPoint::kStart:
      return "start";
    case ast::ContactPoint::kEnd:
      return "end";
  }
  return "";
}

/// Converts ast::JunctionSegmentType to its string representation.
constexpr auto ToString(ast::JunctionSegmentType val) noexcept -> std::string_view {
  switch (val) {
    case ast::JunctionSegmentType::kLane:
      return "lane";
    case ast::JunctionSegmentType::kJoint:
      return "joint";
  }
  return "";
}

/// Converts ast::StripMode to its string representation.
constexpr auto ToString(ast::StripMode val) noexcept -> std::string_view {
  switch (val) {
    case ast::StripMode::kIndependent:
      return "independent";
    case ast::StripMode::kRelative:
      return "relative";
  }
  return "";
}

/// Converts ast::JunctionType to its string representation.
constexpr auto ToString(ast::JunctionType val) noexcept -> std::string_view {
  switch (val) {
    case ast::JunctionType::kCommon:
      return "default";
    case ast::JunctionType::kCrossing:
      return "crossing";
    case ast::JunctionType::kDirect:
      return "direct";
    case ast::JunctionType::kVirtual:
      return "virtual";
  }
  return "";
}

/// Converts ast::LayerType to its string representation.
constexpr auto ToString(ast::LayerType val) noexcept -> std::string_view {
  switch (val) {
    case ast::LayerType::kPermanent:
      return "permanent";
    case ast::LayerType::kTemporary:
      return "temporary";
  }
  return "";
}

/// Converts ast::TunnelType to its string representation.
constexpr auto ToString(ast::TunnelType val) noexcept -> std::string_view {
  switch (val) {
    case ast::TunnelType::kStandard:
      return "standard";
    case ast::TunnelType::kUnderpass:
      return "underpass";
  }
  return "";
}

/// Converts ast::BridgeType to its string representation.
constexpr auto ToString(ast::BridgeType val) noexcept -> std::string_view {
  switch (val) {
    case ast::BridgeType::kBrick:
      return "brick";
    case ast::BridgeType::kConcrete:
      return "concrete";
    case ast::BridgeType::kSteel:
      return "steel";
    case ast::BridgeType::kWood:
      return "wood";
  }
  return "";
}

/// Converts ast::LaneType to its string representation.
constexpr auto ToString(ast::LaneType val) noexcept -> std::string_view {
  switch (val) {
    case ast::LaneType::kHov:
      return "hov";
    case ast::LaneType::kBidirectional:
      return "bidirectional";
    case ast::LaneType::kBiking:
      return "biking";
    case ast::LaneType::kBorder:
      return "border";
    case ast::LaneType::kBus:
      return "bus";
    case ast::LaneType::kConnectingRamp:
      return "connectingRamp";
    case ast::LaneType::kCurb:
      return "curb";
    case ast::LaneType::kDriving:
      return "driving";
    case ast::LaneType::kEntry:
      return "entry";
    case ast::LaneType::kExit:
      return "exit";
    case ast::LaneType::kMedian:
      return "median";
    case ast::LaneType::kMwyEntry:
      return "mwyEntry";
    case ast::LaneType::kMwyExit:
      return "mwyExit";
    case ast::LaneType::kNone:
      return "none";
    case ast::LaneType::kOffRamp:
      return "offRamp";
    case ast::LaneType::kOnRamp:
      return "onRamp";
    case ast::LaneType::kParking:
      return "parking";
    case ast::LaneType::kRail:
      return "rail";
    case ast::LaneType::kRestricted:
      return "restricted";
    case ast::LaneType::kRoadWorks:
      return "roadWorks";
    case ast::LaneType::kShared:
      return "shared";
    case ast::LaneType::kShoulder:
      return "shoulder";
    case ast::LaneType::kSidewalk:
      return "sidewalk";
    case ast::LaneType::kSlipLane:
      return "slipLane";
    case ast::LaneType::kSpecial1:
      return "special1";
    case ast::LaneType::kSpecial2:
      return "special2";
    case ast::LaneType::kSpecial3:
      return "special3";
    case ast::LaneType::kStop:
      return "stop";
    case ast::LaneType::kTaxi:
      return "taxi";
    case ast::LaneType::kTram:
      return "tram";
    case ast::LaneType::kWalking:
      return "walking";
  }
  return "";
}

/// Converts ast::ObjectType to its string representation.
constexpr auto ToString(ast::ObjectType val) noexcept -> std::string_view {
  switch (val) {
    case ast::ObjectType::kNone:
      return "none";
    case ast::ObjectType::kObstacle:
      return "obstacle";
    case ast::ObjectType::kCar:
      return "car";
    case ast::ObjectType::kPole:
      return "pole";
    case ast::ObjectType::kTree:
      return "tree";
    case ast::ObjectType::kVegetation:
      return "vegetation";
    case ast::ObjectType::kBarrier:
      return "barrier";
    case ast::ObjectType::kBuilding:
      return "building";
    case ast::ObjectType::kParkingSpace:
      return "parkingSpace";
    case ast::ObjectType::kPatch:
      return "patch";
    case ast::ObjectType::kRailing:
      return "railing";
    case ast::ObjectType::kTrafficIsland:
      return "trafficIsland";
    case ast::ObjectType::kCrosswalk:
      return "crosswalk";
    case ast::ObjectType::kStreetLamp:
      return "streetLamp";
    case ast::ObjectType::kGantry:
      return "gantry";
    case ast::ObjectType::kSoundBarrier:
      return "soundBarrier";
    case ast::ObjectType::kVan:
      return "van";
    case ast::ObjectType::kBus:
      return "bus";
    case ast::ObjectType::kTrailer:
      return "trailer";
    case ast::ObjectType::kBike:
      return "bike";
    case ast::ObjectType::kMotorbike:
      return "motorbike";
    case ast::ObjectType::kTram:
      return "tram";
    case ast::ObjectType::kTrain:
      return "train";
    case ast::ObjectType::kPedestrian:
      return "pedestrian";
    case ast::ObjectType::kWind:
      return "wind";
    case ast::ObjectType::kRoadMark:
      return "roadMark";
    case ast::ObjectType::kRoadSurface:
      return "roadSurface";
  }
  return "";
}

/// Converts ast::RoadType to its string representation.
constexpr auto ToString(ast::RoadType val) noexcept -> std::string_view {
  switch (val) {
    case ast::RoadType::kUnknown:
      return "unknown";
    case ast::RoadType::kBicycle:
      return "bicycle";
    case ast::RoadType::kLowSpeed:
      return "lowSpeed";
    case ast::RoadType::kPedestrian:
      return "pedestrian";
    case ast::RoadType::kMotorway:
      return "motorway";
    case ast::RoadType::kRural:
      return "rural";
    case ast::RoadType::kTownArterial:
      return "townArterial";
    case ast::RoadType::kTownCollector:
      return "townCollector";
    case ast::RoadType::kTownExpressway:
      return "townExpressway";
    case ast::RoadType::kTownLocal:
      return "townLocal";
    case ast::RoadType::kTownPlayStreet:
      return "townPlayStreet";
    case ast::RoadType::kTownPrivate:
      return "townPrivate";
    case ast::RoadType::kTown:
      return "town";
  }
  return "";
}

}  // namespace strada::parser
