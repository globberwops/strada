#pragma once

#include <filesystem>
#include <strada/ast/opendrive.hpp>
#include <strada/parser/errors.hpp>
#include <string_view>

namespace strada::parser {

auto ParseString(std::string_view xml_content) -> ast::OpenDrive;

auto ParseFile(const std::filesystem::path& file_path) -> ast::OpenDrive;

}  // namespace strada::parser
