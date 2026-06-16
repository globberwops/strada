#pragma once

#include <optional>
#include <string>
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

struct Coefficient {
  double s{};
  double a{};
  double b{};
  double c{};
  double d{};
};

struct CrossSectionSurfaceStrip {
  int id{};
  std::string mode{"independent"};
  std::vector<Coefficient> constant;
  std::vector<Coefficient> linear;
  std::vector<Coefficient> quadratic;
  std::vector<Coefficient> cubic;
  std::vector<Coefficient> width;
};

struct CrossSectionSurface {
  std::vector<Coefficient> t_offset;
  std::vector<CrossSectionSurfaceStrip> strips;
};

struct ElevationProfile {
  std::vector<Elevation> elevations;
};

struct LateralProfile {
  std::vector<Superelevation> superelevations;
  std::vector<Shape> shapes;
  std::optional<CrossSectionSurface> cross_section_surface;
};

}  // namespace strada::ast

