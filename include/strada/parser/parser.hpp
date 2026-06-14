#ifndef STRADA_PARSER_PARSER_HPP_
#define STRADA_PARSER_PARSER_HPP_

#include <filesystem>
#include <strada/ast/opendrive.hpp>
#include <string_view>

namespace strada::parser {

auto ParseString(std::string_view xml_content) -> ast::OpenDrive;

auto ParseFile(const std::filesystem::path& file_path) -> ast::OpenDrive;

}  // namespace strada::parser

#endif  // STRADA_PARSER_PARSER_HPP_
