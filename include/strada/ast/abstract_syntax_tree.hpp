// SPDX-License-Identifier: BSL-1.0

#pragma once

#include <strada/ast/header.hpp>
#include <strada/ast/junction.hpp>
#include <strada/ast/road.hpp>
#include <vector>

namespace strada::ast {

/// Root representation of the ASAM OpenDRIVE map Abstract Syntax Tree (AST).
struct AbstractSyntaxTree {
  Header header;                    ///< General map information from the file header.
  std::vector<Road> roads;          ///< All compiled roads in the network.
  std::vector<Junction> junctions;  ///< All junctions connecting roads in the network.
};

}  // namespace strada::ast
