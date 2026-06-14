#pragma once

#include <strada/ast/header.hpp>
#include <strada/ast/road.hpp>
#include <vector>

namespace strada::ast {

struct OpenDrive {
  Header header_;
  std::vector<Road> roads_;
};

}  // namespace strada::ast
