#ifndef STRADA_AST_ROAD_HPP_
#define STRADA_AST_ROAD_HPP_

#include <strada/ast/geometry.hpp>
#include <strada/ast/lanes.hpp>
#include <strada/ast/profiles.hpp>
#include <string>
#include <vector>

namespace strada::ast {

struct Road {
  std::string id_;
  double length_{};
  std::string junction_ = "-1";
  std::string rule_ = "RHT";
  std::string name_;
  std::vector<GeometryRecord> plan_view_;
  ElevationProfile elevation_profile_;
  LateralProfile lateral_profile_;
  Lanes lanes_;
};

}  // namespace strada::ast

#endif  // STRADA_AST_ROAD_HPP_
