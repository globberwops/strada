#pragma once

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
