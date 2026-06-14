#pragma once

#include <optional>
#include <string>
#include <vector>

namespace strada::ast {

struct LaneOffset {
  double s_{};
  double a_{};
  double b_{};
  double c_{};
  double d_{};
};

struct LaneWidth {
  double s_offset_{};
  double a_{};
  double b_{};
  double c_{};
  double d_{};
};

struct LaneHeight {
  double s_offset_{};
  double inner_{};
  double outer_{};
};

struct Lane {
  int id_{};
  std::string type_;
  bool level_{};
  std::optional<int> predecessor_;
  std::optional<int> successor_;
  std::vector<LaneWidth> widths_;
  std::vector<LaneHeight> heights_;
};

struct LaneSection {
  double s_{};
  std::vector<Lane> left_;
  std::vector<Lane> center_;
  std::vector<Lane> right_;
};

struct Lanes {
  std::vector<LaneOffset> offsets_;
  std::vector<LaneSection> sections_;
};

}  // namespace strada::ast
