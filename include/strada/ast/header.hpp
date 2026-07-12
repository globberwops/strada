#pragma once

#include <strada/ast/extensions.hpp>
#include <string>

namespace strada::ast {

/// General information about the road network map from the file header.
struct Header {
  int rev_major{};            ///< OpenDRIVE major revision number.
  int rev_minor{};            ///< OpenDRIVE minor revision number.
  std::string name;           ///< Name of the database.
  std::string version;        ///< Version number of the database.
  std::string date;           ///< Date/time of database creation.
  double north{};             ///< Maximum north coordinate boundary.
  double south{};             ///< Maximum south coordinate boundary.
  double east{};              ///< Maximum east coordinate boundary.
  double west{};              ///< Maximum west coordinate boundary.
  std::string vendor;         ///< Vendor name of the map exporter.
  std::string geo_reference;  ///< Spatial reference projection description.
  Extensions extensions;      ///< Non-schema and custom user data extensions.
};

}  // namespace strada::ast
