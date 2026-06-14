#pragma once

#include <string>

namespace strada::ast {

struct Header {
  int rev_major{};
  int rev_minor{};
  std::string name;
  std::string version;
  std::string date;
  double north{};
  double south{};
  double east{};
  double west{};
  std::string vendor;
  std::string geo_reference;
};

}  // namespace strada::ast
