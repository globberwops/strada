#include "strada/strada.hpp"

#include <filesystem>
#include <optional>
#include <string_view>

#include "strada/parser/parser.hpp"

namespace strada {

Strada::Strada(const std::filesystem::path& file_path, Options options)
    : ast_(parser::ParseFile(file_path)),
      cpm_(ast_),
      graph_(ast_),
      tessellator_(options.chord_error.has_value()
                       ? std::make_optional<tess::Tessellator>(ast_, cpm_, *options.chord_error)
                       : std::nullopt) {}

Strada::Strada(std::string_view xml_content, Options options)
    : ast_(parser::ParseString(xml_content)),
      cpm_(ast_),
      graph_(ast_),
      tessellator_(options.chord_error.has_value()
                       ? std::make_optional<tess::Tessellator>(ast_, cpm_, *options.chord_error)
                       : std::nullopt) {}

auto Strada::AbstractSyntaxTree() const noexcept -> const ast::AbstractSyntaxTree& { return ast_; }

auto Strada::CompiledPhysicsModel() const noexcept -> const cpm::CompiledPhysicsModel& { return cpm_; }

auto Strada::Graph() const noexcept -> const routing::Graph& { return graph_; }

auto Strada::Tessellator() const noexcept -> std::optional<std::reference_wrapper<const tess::Tessellator>> {
  if (tessellator_.has_value()) {
    return {std::ref(*tessellator_)};
  }
  return std::nullopt;
}

}  // namespace strada
