// SPDX-License-Identifier: BSL-1.0

#pragma once

#include <filesystem>
#include <strada/ast/abstract_syntax_tree.hpp>
#include <strada/parser/errors.hpp>
#include <string_view>

namespace strada::parser {

/// Parses a road network map from an XML string representation.
///
/// \param xml_content The string view of the XODR/XML map payload.
/// \return The parsed abstract syntax tree (AST).
/// \throws XmlParseError If the XML is malformed.
/// \throws ParseError If the AST construction fails due to model invariants.
auto ParseString(std::string_view xml_content) -> ast::AbstractSyntaxTree;

/// Parses a road network map from a specified file on disk.
///
/// \param file_path The path to the XML/XODR file.
/// \return The parsed abstract syntax tree (AST).
/// \throws XmlParseError If the XML is malformed or the file cannot be read.
/// \throws ParseError If the AST construction fails due to model invariants.
auto ParseFile(const std::filesystem::path& file_path) -> ast::AbstractSyntaxTree;

}  // namespace strada::parser

