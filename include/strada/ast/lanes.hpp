#pragma once

#include <optional>
#include <string>
#include <vector>

namespace strada::ast {

struct LaneOffset {
  double s{};
  double a{};
  double b{};
  double c{};
  double d{};
};

struct LaneWidth {
  double s_offset{};
  double a{};
  double b{};
  double c{};
  double d{};
};

struct LaneHeight {
  double s_offset{};
  double inner{};
  double outer{};
};

struct Lane {
  int id{};
  std::string type;
  bool level{};
  std::optional<int> predecessor;
  std::optional<int> successor;
  std::vector<LaneWidth> widths;
  std::vector<LaneHeight> heights;
};

struct LaneSection {
  double s{};
  std::vector<Lane> left;
  std::vector<Lane> center;
  std::vector<Lane> right;
};

struct Lanes {
  std::vector<LaneOffset> offsets;
  std::vector<LaneSection> sections;
};

}  // namespace strada::ast
