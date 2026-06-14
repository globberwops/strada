#include <algorithm>
#include <pugixml.hpp>
#include <stdexcept>
#include <strada/parser/parser.hpp>
#include <string>
#include <vector>

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

    // PlanView Geometries
    pugi::xml_node plan_view_node = road_node.child("planView");
    if (!plan_view_node.empty()) {
      pugi::xml_node geom_node = plan_view_node.child("geometry");
      while (!geom_node.empty()) {
        ast::GeometryRecord geom;
        geom.s_ = geom_node.attribute("s").as_double(0.0);
        geom.x_ = geom_node.attribute("x").as_double(0.0);
        geom.y_ = geom_node.attribute("y").as_double(0.0);
        geom.hdg_ = geom_node.attribute("hdg").as_double(0.0);
        geom.length_ = geom_node.attribute("length").as_double(0.0);

        if (pugi::xml_node line_node = geom_node.child("line"); !line_node.empty()) {
          geom.shape_ = ast::Line{};
        } else if (pugi::xml_node spiral_node = geom_node.child("spiral"); !spiral_node.empty()) {
          ast::Spiral spiral;
          spiral.curv_start_ = spiral_node.attribute("curvStart").as_double(0.0);
          spiral.curv_end_ = spiral_node.attribute("curvEnd").as_double(0.0);
          geom.shape_ = spiral;
        } else if (pugi::xml_node arc_node = geom_node.child("arc"); !arc_node.empty()) {
          ast::Arc arc;
          arc.curvature_ = arc_node.attribute("curvature").as_double(0.0);
          geom.shape_ = arc;
        } else if (pugi::xml_node poly3_node = geom_node.child("poly3"); !poly3_node.empty()) {
          ast::Poly3 poly3;
          poly3.a_ = poly3_node.attribute("a").as_double(0.0);
          poly3.b_ = poly3_node.attribute("b").as_double(0.0);
          poly3.c_ = poly3_node.attribute("c").as_double(0.0);
          poly3.d_ = poly3_node.attribute("d").as_double(0.0);
          geom.shape_ = poly3;
        } else if (pugi::xml_node param_poly3_node = geom_node.child("paramPoly3"); !param_poly3_node.empty()) {
          ast::ParamPoly3 param_poly3;
          param_poly3.a_u_ = param_poly3_node.attribute("aU").as_double(0.0);
          param_poly3.b_u_ = param_poly3_node.attribute("bU").as_double(0.0);
          param_poly3.c_u_ = param_poly3_node.attribute("cU").as_double(0.0);
          param_poly3.d_u_ = param_poly3_node.attribute("dU").as_double(0.0);
          param_poly3.a_v_ = param_poly3_node.attribute("aV").as_double(0.0);
          param_poly3.b_v_ = param_poly3_node.attribute("bV").as_double(0.0);
          param_poly3.c_v_ = param_poly3_node.attribute("cV").as_double(0.0);
          param_poly3.d_v_ = param_poly3_node.attribute("dV").as_double(0.0);

          std::string p_range_str = param_poly3_node.attribute("pRange").as_string("normalized");
          if (p_range_str == "arcLength") {
            param_poly3.p_range_ = ast::PRange::ArcLength;
          } else {
            param_poly3.p_range_ = ast::PRange::Normalized;
          }
          geom.shape_ = param_poly3;
        } else {
          throw std::runtime_error("Unsupported or missing geometry shape type in <geometry>");
        }

        road.plan_view_.push_back(geom);
        geom_node = geom_node.next_sibling("geometry");
      }

      std::ranges::sort(road.plan_view_, [](const ast::GeometryRecord& lhs, const ast::GeometryRecord& rhs) -> bool {
        return lhs.s_ < rhs.s_;
      });
    }

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
