#include <algorithm>
#include <pugixml.hpp>
#include <stdexcept>
#include <strada/parser/parser.hpp>
#include <string>
#include <vector>

namespace strada::parser {

namespace {

auto ParseHeader(pugi::xml_node header_node) -> ast::Header {
  ast::Header header;
  header.rev_major = header_node.attribute("revMajor").as_int(0);
  header.rev_minor = header_node.attribute("revMinor").as_int(0);
  header.name = header_node.attribute("name").as_string("");
  header.version = header_node.attribute("version").as_string("");
  header.date = header_node.attribute("date").as_string("");
  header.north = header_node.attribute("north").as_double(0.0);
  header.south = header_node.attribute("south").as_double(0.0);
  header.east = header_node.attribute("east").as_double(0.0);
  header.west = header_node.attribute("west").as_double(0.0);
  header.vendor = header_node.attribute("vendor").as_string("");

  pugi::xml_node geo_ref_node = header_node.child("geoReference");
  if (!geo_ref_node.empty()) {
    header.geo_reference = geo_ref_node.child_value();
  }
  return header;
}

auto ParseGeometry(pugi::xml_node geom_node) -> ast::GeometryRecord {
  ast::GeometryRecord geom;
  geom.s = geom_node.attribute("s").as_double(0.0);
  geom.x = geom_node.attribute("x").as_double(0.0);
  geom.y = geom_node.attribute("y").as_double(0.0);
  geom.hdg = geom_node.attribute("hdg").as_double(0.0);
  geom.length = geom_node.attribute("length").as_double(0.0);

  if (pugi::xml_node line_node = geom_node.child("line"); !line_node.empty()) {
    geom.shape = ast::Line{};
  } else if (pugi::xml_node spiral_node = geom_node.child("spiral"); !spiral_node.empty()) {
    ast::Spiral spiral;
    spiral.curv_start = spiral_node.attribute("curvStart").as_double(0.0);
    spiral.curv_end = spiral_node.attribute("curvEnd").as_double(0.0);
    geom.shape = spiral;
  } else if (pugi::xml_node arc_node = geom_node.child("arc"); !arc_node.empty()) {
    ast::Arc arc;
    arc.curvature = arc_node.attribute("curvature").as_double(0.0);
    geom.shape = arc;
  } else if (pugi::xml_node poly3_node = geom_node.child("poly3"); !poly3_node.empty()) {
    ast::Poly3 poly3;
    poly3.a = poly3_node.attribute("a").as_double(0.0);
    poly3.b = poly3_node.attribute("b").as_double(0.0);
    poly3.c = poly3_node.attribute("c").as_double(0.0);
    poly3.d = poly3_node.attribute("d").as_double(0.0);
    geom.shape = poly3;
  } else if (pugi::xml_node param_poly3_node = geom_node.child("paramPoly3"); !param_poly3_node.empty()) {
    ast::ParamPoly3 param_poly3;
    param_poly3.a_u = param_poly3_node.attribute("aU").as_double(0.0);
    param_poly3.b_u = param_poly3_node.attribute("bU").as_double(0.0);
    param_poly3.c_u = param_poly3_node.attribute("cU").as_double(0.0);
    param_poly3.d_u = param_poly3_node.attribute("dU").as_double(0.0);
    param_poly3.a_v = param_poly3_node.attribute("aV").as_double(0.0);
    param_poly3.b_v = param_poly3_node.attribute("bV").as_double(0.0);
    param_poly3.c_v = param_poly3_node.attribute("cV").as_double(0.0);
    param_poly3.d_v = param_poly3_node.attribute("dV").as_double(0.0);

    std::string p_range_str = param_poly3_node.attribute("pRange").as_string("normalized");
    if (p_range_str == "arcLength") {
      param_poly3.p_range = ast::PRange::ArcLength;
    } else {
      param_poly3.p_range = ast::PRange::Normalized;
    }
    geom.shape = param_poly3;
  } else {
    throw std::runtime_error("Unsupported or missing geometry shape type in <geometry>");
  }
  return geom;
}

auto ParseElevationProfile(pugi::xml_node elev_prof_node) -> ast::ElevationProfile {
  ast::ElevationProfile profile;
  if (elev_prof_node.empty()) {
    return profile;
  }
  pugi::xml_node elev_node = elev_prof_node.child("elevation");
  while (!elev_node.empty()) {
    ast::Elevation elev;
    elev.s = elev_node.attribute("s").as_double(0.0);
    elev.a = elev_node.attribute("a").as_double(0.0);
    elev.b = elev_node.attribute("b").as_double(0.0);
    elev.c = elev_node.attribute("c").as_double(0.0);
    elev.d = elev_node.attribute("d").as_double(0.0);
    profile.elevations.push_back(elev);
    elev_node = elev_node.next_sibling("elevation");
  }
  return profile;
}

auto ParseLateralProfile(pugi::xml_node lat_prof_node) -> ast::LateralProfile {
  ast::LateralProfile profile;
  if (lat_prof_node.empty()) {
    return profile;
  }
  pugi::xml_node super_node = lat_prof_node.child("superelevation");
  while (!super_node.empty()) {
    ast::Superelevation super;
    super.s = super_node.attribute("s").as_double(0.0);
    super.a = super_node.attribute("a").as_double(0.0);
    super.b = super_node.attribute("b").as_double(0.0);
    super.c = super_node.attribute("c").as_double(0.0);
    super.d = super_node.attribute("d").as_double(0.0);
    profile.superelevations.push_back(super);
    super_node = super_node.next_sibling("superelevation");
  }
  pugi::xml_node shape_node = lat_prof_node.child("shape");
  while (!shape_node.empty()) {
    ast::Shape shape;
    shape.s = shape_node.attribute("s").as_double(0.0);
    shape.t = shape_node.attribute("t").as_double(0.0);
    shape.a = shape_node.attribute("a").as_double(0.0);
    shape.b = shape_node.attribute("b").as_double(0.0);
    shape.c = shape_node.attribute("c").as_double(0.0);
    shape.d = shape_node.attribute("d").as_double(0.0);
    profile.shapes.push_back(shape);
    shape_node = shape_node.next_sibling("shape");
  }
  return profile;
}

auto ParseLane(pugi::xml_node lane_node) -> ast::Lane {
  ast::Lane lane;
  lane.id = lane_node.attribute("id").as_int(0);
  lane.type = lane_node.attribute("type").as_string("");
  lane.level = lane_node.attribute("level").as_bool(false);

  pugi::xml_node link_node = lane_node.child("link");
  if (!link_node.empty()) {
    pugi::xml_node pred_node = link_node.child("predecessor");
    if (!pred_node.empty()) {
      lane.predecessor = pred_node.attribute("id").as_int(0);
    }
    pugi::xml_node succ_node = link_node.child("successor");
    if (!succ_node.empty()) {
      lane.successor = succ_node.attribute("id").as_int(0);
    }
  }

  pugi::xml_node width_node = lane_node.child("width");
  while (!width_node.empty()) {
    ast::LaneWidth width;
    width.s_offset = width_node.attribute("sOffset").as_double(0.0);
    width.a = width_node.attribute("a").as_double(0.0);
    width.b = width_node.attribute("b").as_double(0.0);
    width.c = width_node.attribute("c").as_double(0.0);
    width.d = width_node.attribute("d").as_double(0.0);
    lane.widths.push_back(width);
    width_node = width_node.next_sibling("width");
  }

  pugi::xml_node height_node = lane_node.child("height");
  while (!height_node.empty()) {
    ast::LaneHeight height;
    height.s_offset = height_node.attribute("sOffset").as_double(0.0);
    height.inner = height_node.attribute("inner").as_double(0.0);
    height.outer = height_node.attribute("outer").as_double(0.0);
    lane.heights.push_back(height);
    height_node = height_node.next_sibling("height");
  }
  return lane;
}

auto ParseLanesInContainer(pugi::xml_node container, std::vector<ast::Lane>& lanes_out) -> void {
  if (container.empty()) {
    return;
  }
  pugi::xml_node lane_node = container.child("lane");
  while (!lane_node.empty()) {
    lanes_out.push_back(ParseLane(lane_node));
    lane_node = lane_node.next_sibling("lane");
  }
}

auto ParseLanes(pugi::xml_node lanes_node) -> ast::Lanes {
  ast::Lanes lanes;
  if (lanes_node.empty()) {
    return lanes;
  }
  pugi::xml_node offset_node = lanes_node.child("laneOffset");
  while (!offset_node.empty()) {
    ast::LaneOffset offset;
    offset.s = offset_node.attribute("s").as_double(0.0);
    offset.a = offset_node.attribute("a").as_double(0.0);
    offset.b = offset_node.attribute("b").as_double(0.0);
    offset.c = offset_node.attribute("c").as_double(0.0);
    offset.d = offset_node.attribute("d").as_double(0.0);
    lanes.offsets.push_back(offset);
    offset_node = offset_node.next_sibling("laneOffset");
  }
  pugi::xml_node sec_node = lanes_node.child("laneSection");
  while (!sec_node.empty()) {
    ast::LaneSection section;
    section.s = sec_node.attribute("s").as_double(0.0);
    ParseLanesInContainer(sec_node.child("left"), section.left);
    ParseLanesInContainer(sec_node.child("center"), section.center);
    ParseLanesInContainer(sec_node.child("right"), section.right);
    lanes.sections.push_back(section);
    sec_node = sec_node.next_sibling("laneSection");
  }
  return lanes;
}

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
  opendrive.header = ParseHeader(header_node);

  pugi::xml_node road_node = root.child("road");
  while (!road_node.empty()) {
    ast::Road road;
    road.id = road_node.attribute("id").as_string("");
    road.length = road_node.attribute("length").as_double(0.0);
    road.junction = road_node.attribute("junction").as_string("-1");
    std::string rule_str = road_node.attribute("rule").as_string("RHT");
    if (rule_str == "LHT") {
      road.rule = ast::TrafficRule::LHT;
    } else {
      road.rule = ast::TrafficRule::RHT;
    }
    road.name = road_node.attribute("name").as_string("");

    // PlanView Geometries
    pugi::xml_node plan_view_node = road_node.child("planView");
    if (!plan_view_node.empty()) {
      pugi::xml_node geom_node = plan_view_node.child("geometry");
      while (!geom_node.empty()) {
        road.plan_view.push_back(ParseGeometry(geom_node));
        geom_node = geom_node.next_sibling("geometry");
      }
      std::ranges::sort(road.plan_view, [](const ast::GeometryRecord& lhs, const ast::GeometryRecord& rhs) -> bool {
        return lhs.s < rhs.s;
      });
    }

    // Elevation & Lateral Profiles
    road.elevation_profile = ParseElevationProfile(road_node.child("elevationProfile"));
    road.lateral_profile = ParseLateralProfile(road_node.child("lateralProfile"));

    // Lanes
    road.lanes = ParseLanes(road_node.child("lanes"));
    std::ranges::sort(road.lanes.sections,
                      [](const ast::LaneSection& lhs, const ast::LaneSection& rhs) -> bool { return lhs.s < rhs.s; });

    opendrive.roads.push_back(road);
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
