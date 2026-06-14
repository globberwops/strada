#ifndef STRADA_AST_OPENDRIVE_HPP_
#define STRADA_AST_OPENDRIVE_HPP_

#include <strada/ast/header.hpp>
#include <strada/ast/road.hpp>
#include <vector>

namespace strada::ast {

struct OpenDrive {
  Header header_;
  std::vector<Road> roads_;
};

}  // namespace strada::ast

#endif  // STRADA_AST_OPENDRIVE_HPP_
