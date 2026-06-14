#pragma once

#include <vector>

namespace strada::ast {

struct Elevation {
  double s{};
  double a{};
  double b{};
  double c{};
  double d{};
};

struct Superelevation {
  double s{};
  double a{};
  double b{};
  double c{};
  double d{};
};

struct Shape {
  double s{};
  double t{};
  double a{};
  double b{};
  double c{};
  double d{};
};

struct ElevationProfile {
  std::vector<Elevation> elevations;
};

struct LateralProfile {
  std::vector<Superelevation> superelevations;
  std::vector<Shape> shapes;
};

}  // namespace strada::ast
