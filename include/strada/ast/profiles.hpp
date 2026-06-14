#ifndef STRADA_AST_PROFILES_HPP_
#define STRADA_AST_PROFILES_HPP_

#include <vector>

namespace strada::ast {

struct Elevation {
  double s_{};
  double a_{};
  double b_{};
  double c_{};
  double d_{};
};

struct Superelevation {
  double s_{};
  double a_{};
  double b_{};
  double c_{};
  double d_{};
};

struct Shape {
  double s_{};
  double t_{};
  double a_{};
  double b_{};
  double c_{};
  double d_{};
};

struct ElevationProfile {
  std::vector<Elevation> elevations_;
};

struct LateralProfile {
  std::vector<Superelevation> superelevations_;
  std::vector<Shape> shapes_;
};

}  // namespace strada::ast

#endif  // STRADA_AST_PROFILES_HPP_
