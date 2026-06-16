#pragma once

#include <filesystem>
#include <strada/ast/abstract_syntax_tree.hpp>
#include <strada/parser/errors.hpp>
#include <string_view>

namespace strada::parser {

auto ParseString(std::string_view xml_content) -> ast::AbstractSyntaxTree;

auto ParseFile(const std::filesystem::path& file_path) -> ast::AbstractSyntaxTree;

}  // namespace strada::parser
