#pragma once

#include <map>
#include <string>
#include <vector>

namespace strada::ast {

/// Holds extensibility data for an OpenDRIVE AST node.
/// - `attributes`  captures non-schema XML attributes as key-value pairs.
/// - `user_data`   captures the raw XML string content of any <userData> child
///                 element, preserving vendor-specific sub-trees verbatim.
struct Extensions {
  std::map<std::string, std::string> attributes;
  std::vector<std::string> user_data;
};

}  // namespace strada::ast
