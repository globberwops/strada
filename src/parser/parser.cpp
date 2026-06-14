#include <pugixml.hpp>
#include <stdexcept>
#include <strada/parser/parser.hpp>
#include <string>

namespace strada::parser {

namespace {

auto ParseDocument(const pugi::xml_document& doc) -> ast::OpenDrive {
  pugi::xml_node root = doc.child("OpenDRIVE");
  if (!root) {
    throw std::runtime_error("Missing <OpenDRIVE> root element");
  }

  pugi::xml_node header_node = root.child("header");
  if (!header_node) {
    throw std::runtime_error("Missing <header> element");
  }

  ast::OpenDrive opendrive;
  auto& header = opendrive.header_;

  header.rev_major_ = header_node.attribute("revMajor").as_int(0);
  header.rev_minor_ = header_node.attribute("revMinor").as_int(0);
  header.name_ = header_node.attribute("name").as_string("");
  header.version_ = header_node.attribute("version").as_string("");
  header.date_ = header_node.attribute("date").as_string("");
  header.north_ = header_node.attribute("north").as_double(0.0);
  header.south_ = header_node.attribute("south").as_double(0.0);
  header.east_ = header_node.attribute("east").as_double(0.0);
  header.west_ = header_node.attribute("west").as_double(0.0);
  header.vendor_ = header_node.attribute("vendor").as_string("");

  pugi::xml_node geo_ref_node = header_node.child("geoReference");
  if (!geo_ref_node.empty()) {
    header.geo_reference_ = geo_ref_node.child_value();
  }

  pugi::xml_node road_node = root.child("road");
  while (!road_node.empty()) {
    ast::Road road;
    road.id_ = road_node.attribute("id").as_string("");
    road.length_ = road_node.attribute("length").as_double(0.0);
    road.junction_ = road_node.attribute("junction").as_string("-1");
    road.rule_ = road_node.attribute("rule").as_string("RHT");
    road.name_ = road_node.attribute("name").as_string("");
    opendrive.roads_.push_back(road);
    road_node = road_node.next_sibling("road");
  }

  return opendrive;
}

}  // namespace

auto ParseString(std::string_view xml_content) -> ast::OpenDrive {
  pugi::xml_document doc;
  pugi::xml_parse_result result = doc.load_buffer(xml_content.data(), xml_content.size());
  if (!result) {
    throw std::runtime_error(std::string("Failed to parse XML from string: ") + result.description());
  }
  return ParseDocument(doc);
}

auto ParseFile(const std::filesystem::path& file_path) -> ast::OpenDrive {
  pugi::xml_document doc;
  pugi::xml_parse_result result = doc.load_file(file_path.c_str());
  if (!result) {
    throw std::runtime_error(std::string("Failed to parse XML from file: ") + result.description());
  }
  return ParseDocument(doc);
}

}  // namespace strada::parser
