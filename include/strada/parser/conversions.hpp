// SPDX-License-Identifier: BSL-1.0

#pragma once

#include <optional>
#include <strada/ast/geometry.hpp>
#include <strada/ast/junction.hpp>
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

/// Converts ast::TrafficRule to its string representation.
inline constexpr auto ToString(ast::TrafficRule val) noexcept -> std::string_view {
  switch (val) {
    case ast::TrafficRule::kRht:
      return "RHT";
    case ast::TrafficRule::kLht:
      return "LHT";
  }
  return "";
}

/// Converts ast::Orientation to its string representation.
inline constexpr auto ToString(ast::Orientation val) noexcept -> std::string_view {
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
inline constexpr auto ToString(ast::PRange val) noexcept -> std::string_view {
  switch (val) {
    case ast::PRange::kNormalized:
      return "normalized";
    case ast::PRange::kArcLength:
      return "arcLength";
  }
  return "";
}

/// Converts ast::ContactPoint to its string representation.
inline constexpr auto ToString(ast::ContactPoint val) noexcept -> std::string_view {
  switch (val) {
    case ast::ContactPoint::kStart:
      return "start";
    case ast::ContactPoint::kEnd:
      return "end";
  }
  return "";
}

/// Converts ast::JunctionSegmentType to its string representation.
inline constexpr auto ToString(ast::JunctionSegmentType val) noexcept -> std::string_view {
  switch (val) {
    case ast::JunctionSegmentType::kLane:
      return "lane";
    case ast::JunctionSegmentType::kJoint:
      return "joint";
  }
  return "";
}

/// Converts ast::StripMode to its string representation.
inline constexpr auto ToString(ast::StripMode val) noexcept -> std::string_view {
  switch (val) {
    case ast::StripMode::kIndependent:
      return "independent";
    case ast::StripMode::kRelative:
      return "relative";
  }
  return "";
}

}  // namespace strada::parser
