#pragma once

#include <cstdint>
#include <strada/ast/extensions.hpp>
#include <string>
#include <vector>

namespace strada::ast {

enum class ContactPoint : std::uint8_t { kStart, kEnd };

struct LaneLink {
  int from{};
  int to{};
};

struct Connection {
  std::string id;
  std::string incoming_road;
  std::string connecting_road;
  ContactPoint contact_point = ContactPoint::kStart;
  std::vector<LaneLink> lane_links;
};

struct Junction {
  std::string id;
  std::string name;
  std::string type;
  std::vector<Connection> connections;
  Extensions extensions;
};

}  // namespace strada::ast
