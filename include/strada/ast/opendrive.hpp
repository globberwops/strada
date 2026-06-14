#pragma once

#include <strada/ast/header.hpp>
#include <strada/ast/junction.hpp>
#include <strada/ast/road.hpp>
#include <vector>

namespace strada::ast {

struct OpenDrive {
  Header header;
  std::vector<Road> roads;
  std::vector<Junction> junctions;
};

}  // namespace strada::ast
