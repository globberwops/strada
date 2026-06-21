// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <map>
#include <string>
#include <vector>

namespace strada::ast {

/// Holds extensibility data for an OpenDRIVE AST node.
///
/// This structure captures non-schema XML attributes and vendor-specific userData elements.
struct Extensions {
  std::map<std::string, std::string> attributes;  ///< Key-value pairs for non-schema XML attributes.
  std::vector<std::string> user_data;             ///< Raw XML string contents of <userData> elements.
};

}  // namespace strada::ast
