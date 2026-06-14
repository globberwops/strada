#ifndef STRADA_AST_ROAD_HPP_
#define STRADA_AST_ROAD_HPP_

#include <string>

namespace strada::ast {

struct Road {
  std::string id_;
  double length_{};
  std::string junction_ = "-1";
  std::string rule_ = "RHT";
  std::string name_;
};

}  // namespace strada::ast

#endif  // STRADA_AST_ROAD_HPP_
