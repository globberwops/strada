#include <algorithm>
#include <limits>
#include <map>
#include <pugixml.hpp>
#include <sstream>
#include <strada/ast/objects.hpp>
#include <strada/parser/conversions.hpp>
#include <strada/parser/errors.hpp>
#include <strada/parser/parser.hpp>
#include <string>
#include <unordered_set>
#include <vector>

namespace strada::parser {

namespace {

auto NodeToString(const pugi::xml_node& node) -> std::string {
  std::ostringstream oss;
  node.print(oss, "  ", pugi::format_default);
  return oss.str();
}

auto ParseExtensions(const pugi::xml_node& node, const std::unordered_set<std::string>& known_attrs)
    -> ast::Extensions {
  ast::Extensions ext;
  for (const pugi::xml_attribute& attr : node.attributes()) {
    if (!known_attrs.contains(attr.name())) {
      ext.attributes.emplace(attr.name(), attr.value());
    }
  }
  pugi::xml_node child = node.child("userData");
  while (!child.empty()) {
    ext.user_data.push_back(NodeToString(child));
    child = child.next_sibling("userData");
  }
  return ext;
}

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

  const pugi::xml_node geo_ref_node = header_node.child("geoReference");
  if (!geo_ref_node.empty()) {
    header.geo_reference = geo_ref_node.child_value();
  }
  static const std::unordered_set<std::string> kNownHeaderAttrs = {"revMajor", "revMinor", "name", "version", "date",
                                                                   "north",    "south",    "east", "west",    "vendor"};
  header.extensions = ParseExtensions(header_node, kNownHeaderAttrs);
  return header;
}

auto ParseGeometry(pugi::xml_node geom_node) -> ast::GeometryRecord {
  ast::GeometryRecord geom;
  geom.s = geom_node.attribute("s").as_double(0.0);
  geom.x = geom_node.attribute("x").as_double(0.0);
  geom.y = geom_node.attribute("y").as_double(0.0);
  geom.hdg = geom_node.attribute("hdg").as_double(0.0);
  geom.length = geom_node.attribute("length").as_double(0.0);

  if (const pugi::xml_node line_node = geom_node.child("line"); !line_node.empty()) {
    geom.shape = ast::Line{};
  } else if (const pugi::xml_node spiral_node = geom_node.child("spiral"); !spiral_node.empty()) {
    ast::Spiral spiral;
    spiral.curv_start = spiral_node.attribute("curvStart").as_double(0.0);
    spiral.curv_end = spiral_node.attribute("curvEnd").as_double(0.0);
    geom.shape = spiral;
  } else if (const pugi::xml_node arc_node = geom_node.child("arc"); !arc_node.empty()) {
    ast::Arc arc;
    arc.curvature = arc_node.attribute("curvature").as_double(0.0);
    geom.shape = arc;
  } else if (const pugi::xml_node poly3_node = geom_node.child("poly3"); !poly3_node.empty()) {
    ast::Poly3 poly3;
    poly3.a = poly3_node.attribute("a").as_double(0.0);
    poly3.b = poly3_node.attribute("b").as_double(0.0);
    poly3.c = poly3_node.attribute("c").as_double(0.0);
    poly3.d = poly3_node.attribute("d").as_double(0.0);
    geom.shape = poly3;
  } else if (const pugi::xml_node param_poly3_node = geom_node.child("paramPoly3"); !param_poly3_node.empty()) {
    ast::ParamPoly3 param_poly3;
    param_poly3.a_u = param_poly3_node.attribute("aU").as_double(0.0);
    param_poly3.b_u = param_poly3_node.attribute("bU").as_double(0.0);
    param_poly3.c_u = param_poly3_node.attribute("cU").as_double(0.0);
    param_poly3.d_u = param_poly3_node.attribute("dU").as_double(0.0);
    param_poly3.a_v = param_poly3_node.attribute("aV").as_double(0.0);
    param_poly3.b_v = param_poly3_node.attribute("bV").as_double(0.0);
    param_poly3.c_v = param_poly3_node.attribute("cV").as_double(0.0);
    param_poly3.d_v = param_poly3_node.attribute("dV").as_double(0.0);

    const std::string_view p_range_str = param_poly3_node.attribute("pRange").as_string("normalized");
    if (const auto range_opt = FromString<ast::PRange>(p_range_str)) {
      param_poly3.p_range = *range_opt;
    } else {
      throw InvalidAttributeError("<paramPoly3> has invalid pRange=\"" + std::string(p_range_str) + "\"");
    }
    geom.shape = param_poly3;
  } else {
    throw InvalidAttributeError("Unsupported or missing geometry shape type in <geometry>");
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

auto ParseCoefficient(pugi::xml_node coeff_node) -> ast::Coefficient {
  ast::Coefficient coeff;
  coeff.s = coeff_node.attribute("s").as_double(0.0);
  coeff.a = coeff_node.attribute("a").as_double(0.0);
  coeff.b = coeff_node.attribute("b").as_double(0.0);
  coeff.c = coeff_node.attribute("c").as_double(0.0);
  coeff.d = coeff_node.attribute("d").as_double(0.0);
  return coeff;
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

  const pugi::xml_node css_node = lat_prof_node.child("crossSectionSurface");
  if (!css_node.empty()) {
    ast::CrossSectionSurface css;
    const pugi::xml_node t_offset_node = css_node.child("tOffset");
    if (!t_offset_node.empty()) {
      pugi::xml_node coeff_node = t_offset_node.child("coefficients");
      while (!coeff_node.empty()) {
        css.t_offset.push_back(ParseCoefficient(coeff_node));
        coeff_node = coeff_node.next_sibling("coefficients");
      }
    }
    const pugi::xml_node surface_strips_node = css_node.child("surfaceStrips");
    if (!surface_strips_node.empty()) {
      pugi::xml_node strip_node = surface_strips_node.child("strip");
      while (!strip_node.empty()) {
        ast::CrossSectionSurfaceStrip strip;
        strip.id = strip_node.attribute("id").as_int(0);
        const pugi::xml_attribute mode_attr = strip_node.attribute("mode");
        if (mode_attr.empty()) {
          strip.mode = ast::StripMode::kIndependent;
        } else {
          const std::string_view mode_str = mode_attr.as_string();
          if (const auto mode_opt = FromString<ast::StripMode>(mode_str)) {
            strip.mode = *mode_opt;
          } else {
            throw InvalidAttributeError("<strip id=\"" + std::to_string(strip.id) + "\"> has invalid mode=\"" +
                                        std::string(mode_str) + "\"");
          }
        }

        const pugi::xml_node width_node = strip_node.child("width");
        if (!width_node.empty()) {
          pugi::xml_node coeff_node = width_node.child("coefficients");
          while (!coeff_node.empty()) {
            strip.width.push_back(ParseCoefficient(coeff_node));
            coeff_node = coeff_node.next_sibling("coefficients");
          }
        }
        const pugi::xml_node const_node = strip_node.child("constant");
        if (!const_node.empty()) {
          pugi::xml_node coeff_node = const_node.child("coefficients");
          while (!coeff_node.empty()) {
            strip.constant.push_back(ParseCoefficient(coeff_node));
            coeff_node = coeff_node.next_sibling("coefficients");
          }
        }
        const pugi::xml_node linear_node = strip_node.child("linear");
        if (!linear_node.empty()) {
          pugi::xml_node coeff_node = linear_node.child("coefficients");
          while (!coeff_node.empty()) {
            strip.linear.push_back(ParseCoefficient(coeff_node));
            coeff_node = coeff_node.next_sibling("coefficients");
          }
        }
        const pugi::xml_node quad_node = strip_node.child("quadratic");
        if (!quad_node.empty()) {
          pugi::xml_node coeff_node = quad_node.child("coefficients");
          while (!coeff_node.empty()) {
            strip.quadratic.push_back(ParseCoefficient(coeff_node));
            coeff_node = coeff_node.next_sibling("coefficients");
          }
        }
        const pugi::xml_node cubic_node = strip_node.child("cubic");
        if (!cubic_node.empty()) {
          pugi::xml_node coeff_node = cubic_node.child("coefficients");
          while (!coeff_node.empty()) {
            strip.cubic.push_back(ParseCoefficient(coeff_node));
            coeff_node = coeff_node.next_sibling("coefficients");
          }
        }
        css.strips.push_back(strip);
        strip_node = strip_node.next_sibling("strip");
      }
    }
    profile.cross_section_surface = css;
  }

  return profile;
}

auto ParseLane(pugi::xml_node lane_node) -> ast::Lane {
  ast::Lane lane;
  lane.id = lane_node.attribute("id").as_int(0);

  const auto type_attr = lane_node.attribute("type");
  if (type_attr != nullptr) {
    const std::string_view type_str = type_attr.value();
    const auto lane_type = FromString<ast::LaneType>(type_str);
    if (!lane_type) {
      throw InvalidAttributeError("<lane id=\"" + std::to_string(lane.id) + "\"> has invalid type=\"" +
                                  std::string(type_str) + "\"");
    }
    lane.type = *lane_type;
  } else {
    lane.type = ast::LaneType::kNone;
  }

  lane.level = lane_node.attribute("level").as_bool(false);

  const pugi::xml_node link_node = lane_node.child("link");
  if (!link_node.empty()) {
    const pugi::xml_node pred_node = link_node.child("predecessor");
    if (!pred_node.empty()) {
      if (!pred_node.attribute("id")) {
        throw MissingElementError("<predecessor> element inside lane link is missing mandatory 'id' attribute");
      }
      lane.predecessor = pred_node.attribute("id").as_int();
    }
    const pugi::xml_node succ_node = link_node.child("successor");
    if (!succ_node.empty()) {
      if (!succ_node.attribute("id")) {
        throw MissingElementError("<successor> element inside lane link is missing mandatory 'id' attribute");
      }
      lane.successor = succ_node.attribute("id").as_int();
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

auto ParseBoundaryCoordinate(const std::string& str) -> double {
  if (str == "begin" || str == "start") {
    return 0.0;
  }
  if (str == "end") {
    return std::numeric_limits<double>::infinity();
  }
  try {
    return std::stod(str);
  } catch (...) {
    return 0.0;
  }
}

auto ParseBoundary(pugi::xml_node boundary_node) -> ast::JunctionBoundary {
  ast::JunctionBoundary boundary;

  pugi::xml_node seg_node = boundary_node.child("segment");
  while (!seg_node.empty()) {
    ast::JunctionBoundarySegment segment;
    const auto type_attr = seg_node.attribute("type");
    if (!type_attr) {
      segment.type = ast::JunctionSegmentType::kLane;
    } else {
      const std::string_view type_str = type_attr.value();
      if (const auto type_opt = FromString<ast::JunctionSegmentType>(type_str)) {
        segment.type = *type_opt;
      } else {
        throw InvalidAttributeError("<segment> has invalid type=\"" + std::string(type_str) + "\"");
      }
    }

    segment.road_id = seg_node.attribute("roadId").as_string("");

    if (segment.type == ast::JunctionSegmentType::kLane) {
      if (seg_node.attribute("boundaryLane") != nullptr) {
        segment.boundary_lane = seg_node.attribute("boundaryLane").as_int();
      }
      segment.s_start = ParseBoundaryCoordinate(seg_node.attribute("sStart").as_string("begin"));
      segment.s_end = ParseBoundaryCoordinate(seg_node.attribute("sEnd").as_string("end"));
    } else {
      const std::string_view cp_str = seg_node.attribute("contactPoint").as_string("start");
      if (const auto cp_opt = FromString<ast::ContactPoint>(cp_str)) {
        segment.contact_point = *cp_opt;
      } else {
        throw InvalidAttributeError("<segment> has invalid contactPoint=\"" + std::string(cp_str) + "\"");
      }
      if (seg_node.attribute("jointLaneStart") != nullptr) {
        segment.joint_lane_start = seg_node.attribute("jointLaneStart").as_int();
      }
      if (seg_node.attribute("jointLaneEnd") != nullptr) {
        segment.joint_lane_end = seg_node.attribute("jointLaneEnd").as_int();
      }
      segment.transition_length = seg_node.attribute("transitionLength").as_double(0.0);
    }

    static const std::unordered_set<std::string> kKnownSegmentAttrs = {
        "type",         "roadId",         "boundaryLane", "sStart",          "sEnd",
        "contactPoint", "jointLaneStart", "jointLaneEnd", "transitionLength"};
    segment.extensions = ParseExtensions(seg_node, kKnownSegmentAttrs);

    boundary.segments.push_back(segment);
    seg_node = seg_node.next_sibling("segment");
  }

  static const std::unordered_set<std::string> kKnownBoundaryAttrs;
  boundary.extensions = ParseExtensions(boundary_node, kKnownBoundaryAttrs);

  return boundary;
}

auto ParseJunction(pugi::xml_node junction_node) -> ast::Junction {
  ast::Junction junction;
  junction.id = junction_node.attribute("id").as_string("");
  if (junction_node.attribute("name") != nullptr) {
    junction.name = junction_node.attribute("name").as_string();
  } else {
    junction.name = std::nullopt;
  }

  if (junction_node.attribute("type") != nullptr) {
    const std::string_view type_str = junction_node.attribute("type").value();
    if (const auto type_opt = FromString<ast::JunctionType>(type_str)) {
      junction.type = *type_opt;
    } else {
      throw InvalidAttributeError("Invalid junction type: " + std::string(type_str));
    }
  } else {
    junction.type = ast::JunctionType::kCommon;
  }

  pugi::xml_node conn_node = junction_node.child("connection");
  while (!conn_node.empty()) {
    ast::Connection conn;
    conn.id = conn_node.attribute("id").as_string("");
    conn.incoming_road = conn_node.attribute("incomingRoad").as_string("");
    conn.connecting_road = conn_node.attribute("connectingRoad").as_string("");
    const std::string_view cp_str = conn_node.attribute("contactPoint").as_string("start");
    if (const auto cp_opt = FromString<ast::ContactPoint>(cp_str)) {
      conn.contact_point = *cp_opt;
    } else {
      throw InvalidAttributeError("<connection id=\"" + conn.id + "\"> has invalid contactPoint=\"" +
                                  std::string(cp_str) + "\"");
    }

    pugi::xml_node ll_node = conn_node.child("laneLink");
    while (!ll_node.empty()) {
      ast::LaneLink lane_link;
      lane_link.from = ll_node.attribute("from").as_int(0);
      lane_link.to = ll_node.attribute("to").as_int(0);
      conn.lane_links.push_back(lane_link);
      ll_node = ll_node.next_sibling("laneLink");
    }

    junction.connections.push_back(conn);
    conn_node = conn_node.next_sibling("connection");
  }

  const pugi::xml_node boundary_node = junction_node.child("boundary");
  if (!boundary_node.empty()) {
    junction.boundary = ParseBoundary(boundary_node);
  }

  static const std::unordered_set<std::string> kNownJunctionAttrs = {"id", "name", "type"};
  junction.extensions = ParseExtensions(junction_node, kNownJunctionAttrs);
  return junction;
}

auto ParseLaneValidities(pugi::xml_node parent_node) -> std::vector<ast::LaneValidity> {
  std::vector<ast::LaneValidity> validities;
  std::string parent_id = parent_node.attribute("id").as_string();
  std::string parent_name = parent_node.name();

  pugi::xml_node validity_node = parent_node.child("validity");
  while (!validity_node.empty()) {
    ast::LaneValidity validity;
    if (!validity_node.attribute("fromLane")) {
      throw MissingElementError("<validity> element under <" + parent_name + " id=\"" + parent_id +
                                "\"> is missing mandatory 'fromLane' attribute");
    }
    if (!validity_node.attribute("toLane")) {
      throw MissingElementError("<validity> element under <" + parent_name + " id=\"" + parent_id +
                                "\"> is missing mandatory 'toLane' attribute");
    }

    validity.from_lane = validity_node.attribute("fromLane").as_int();
    validity.to_lane = validity_node.attribute("toLane").as_int();

    if (validity.from_lane > validity.to_lane) {
      throw InvalidAttributeError("<validity> element under <" + parent_name + " id=\"" + parent_id +
                                  "\"> has invalid fromLane=" + std::to_string(validity.from_lane) +
                                  " greater than toLane=" + std::to_string(validity.to_lane));
    }

    if (validity_node.attribute("layer") != nullptr) {
      const std::string_view layer_str = validity_node.attribute("layer").value();
      if (const auto layer_opt = FromString<ast::LayerType>(layer_str)) {
        validity.layer = *layer_opt;
      } else {
        throw InvalidAttributeError("<validity> element under <" + parent_name + " id=\"" + parent_id +
                                    "\"> has invalid layer=\"" + std::string(layer_str) + "\"");
      }
    } else {
      validity.layer = ast::LayerType::kPermanent;
    }

    validities.push_back(validity);
    validity_node = validity_node.next_sibling("validity");
  }
  return validities;
}

auto ParseBridge(pugi::xml_node bridge_node) -> ast::Bridge {
  if (!bridge_node.attribute("id")) {
    throw MissingElementError("<bridge> element is missing mandatory 'id' attribute");
  }
  std::string bridge_id = bridge_node.attribute("id").as_string();
  if (!bridge_node.attribute("s")) {
    throw MissingElementError("<bridge id=\"" + bridge_id + "\"> is missing mandatory 's' attribute");
  }
  if (!bridge_node.attribute("length")) {
    throw MissingElementError("<bridge id=\"" + bridge_id + "\"> is missing mandatory 'length' attribute");
  }
  if (!bridge_node.attribute("type")) {
    throw MissingElementError("<bridge id=\"" + bridge_id + "\"> is missing mandatory 'type' attribute");
  }

  double s{bridge_node.attribute("s").as_double()};
  if (s < 0.0) {
    throw InvalidAttributeError("<bridge id=\"" + bridge_id + "\"> has invalid negative 's' attribute");
  }
  double length{bridge_node.attribute("length").as_double()};
  if (length < 0.0) {
    throw InvalidAttributeError("<bridge id=\"" + bridge_id + "\"> has invalid negative 'length' attribute");
  }

  ast::Bridge bridge;
  bridge.id = bridge_id;
  bridge.s = s;
  bridge.length = length;
  if (bridge_node.attribute("name") != nullptr) {
    bridge.name = bridge_node.attribute("name").as_string();
  } else {
    bridge.name = std::nullopt;
  }

  const std::string_view type_str = bridge_node.attribute("type").value();
  if (const auto type_opt = FromString<ast::BridgeType>(type_str)) {
    bridge.type = *type_opt;
  } else {
    throw InvalidAttributeError("<bridge id=\"" + bridge_id + "\"> has invalid type=\"" + std::string(type_str) + "\"");
  }

  bridge.validities = ParseLaneValidities(bridge_node);

  static const std::unordered_set<std::string> kKnownBridgeAttrs = {"id", "s", "length", "name", "type"};
  bridge.extensions = ParseExtensions(bridge_node, kKnownBridgeAttrs);

  return bridge;
}

auto ParseTunnel(pugi::xml_node tunnel_node) -> ast::Tunnel {
  if (!tunnel_node.attribute("id")) {
    throw MissingElementError("<tunnel> element is missing mandatory 'id' attribute");
  }
  std::string tunnel_id = tunnel_node.attribute("id").as_string();
  if (!tunnel_node.attribute("s")) {
    throw MissingElementError("<tunnel id=\"" + tunnel_id + "\"> is missing mandatory 's' attribute");
  }
  if (!tunnel_node.attribute("length")) {
    throw MissingElementError("<tunnel id=\"" + tunnel_id + "\"> is missing mandatory 'length' attribute");
  }
  if (!tunnel_node.attribute("type")) {
    throw MissingElementError("<tunnel id=\"" + tunnel_id + "\"> is missing mandatory 'type' attribute");
  }

  double s{tunnel_node.attribute("s").as_double()};
  if (s < 0.0) {
    throw InvalidAttributeError("<tunnel id=\"" + tunnel_id + "\"> has invalid negative 's' attribute");
  }
  double length{tunnel_node.attribute("length").as_double()};
  if (length < 0.0) {
    throw InvalidAttributeError("<tunnel id=\"" + tunnel_id + "\"> has invalid negative 'length' attribute");
  }

  ast::Tunnel tunnel;
  tunnel.id = tunnel_id;
  tunnel.s = s;
  tunnel.length = length;
  if (tunnel_node.attribute("name") != nullptr) {
    tunnel.name = tunnel_node.attribute("name").as_string();
  } else {
    tunnel.name = std::nullopt;
  }

  const std::string_view type_str = tunnel_node.attribute("type").value();
  if (const auto type_opt = FromString<ast::TunnelType>(type_str)) {
    tunnel.type = *type_opt;
  } else {
    throw InvalidAttributeError("<tunnel id=\"" + tunnel_id + "\"> has invalid type=\"" + std::string(type_str) + "\"");
  }

  if (tunnel_node.attribute("lighting") != nullptr) {
    tunnel.lighting = tunnel_node.attribute("lighting").as_double();
  } else {
    tunnel.lighting = std::nullopt;
  }
  if (tunnel_node.attribute("daylight") != nullptr) {
    tunnel.daylight = tunnel_node.attribute("daylight").as_double();
  } else {
    tunnel.daylight = std::nullopt;
  }
  tunnel.validities = ParseLaneValidities(tunnel_node);

  static const std::unordered_set<std::string> kKnownTunnelAttrs = {"id",   "s",        "length",  "name",
                                                                    "type", "lighting", "daylight"};
  tunnel.extensions = ParseExtensions(tunnel_node, kKnownTunnelAttrs);

  return tunnel;
}

auto ParseSignalDependency(pugi::xml_node dep_node, const std::string& signal_id) -> ast::SignalDependency {
  if (!dep_node.attribute("id")) {
    throw MissingElementError("<dependency> element under <signal id=\"" + signal_id +
                              "\"> is missing mandatory 'id' attribute");
  }
  ast::SignalDependency dep;
  dep.id = dep_node.attribute("id").as_string();
  dep.type = dep_node.attribute("type").as_string("");
  return dep;
}

auto ParseSignalReferenceElement(pugi::xml_node ref_node, const std::string& signal_id) -> ast::SignalReferenceElement {
  if (!ref_node.attribute("elementType")) {
    throw MissingElementError("<reference> element under <signal id=\"" + signal_id +
                              "\"> is missing mandatory 'elementType' attribute");
  }
  if (!ref_node.attribute("elementId")) {
    throw MissingElementError("<reference> element under <signal id=\"" + signal_id +
                              "\"> is missing mandatory 'elementId' attribute");
  }
  ast::SignalReferenceElement ref;
  ref.element_type = ref_node.attribute("elementType").as_string();
  ref.element_id = ref_node.attribute("elementId").as_string();
  return ref;
}

auto ParseSignal(pugi::xml_node signal_node) -> ast::Signal {
  if (!signal_node.attribute("id")) {
    throw MissingElementError("<signal> element is missing mandatory 'id' attribute");
  }
  std::string signal_id = signal_node.attribute("id").as_string();

  if (!signal_node.attribute("s")) {
    throw MissingElementError("<signal id=\"" + signal_id + "\"> is missing mandatory 's' attribute");
  }
  if (!signal_node.attribute("t")) {
    throw MissingElementError("<signal id=\"" + signal_id + "\"> is missing mandatory 't' attribute");
  }
  if (!signal_node.attribute("orientation")) {
    throw MissingElementError("<signal id=\"" + signal_id + "\"> is missing mandatory 'orientation' attribute");
  }
  if (!signal_node.attribute("type")) {
    throw MissingElementError("<signal id=\"" + signal_id + "\"> is missing mandatory 'type' attribute");
  }
  if (!signal_node.attribute("subtype")) {
    throw MissingElementError("<signal id=\"" + signal_id + "\"> is missing mandatory 'subtype' attribute");
  }

  double s = signal_node.attribute("s").as_double();
  if (s < 0.0) {
    throw InvalidAttributeError("<signal id=\"" + signal_id + "\"> has invalid negative 's' attribute");
  }
  double t = signal_node.attribute("t").as_double();

  ast::Signal signal;
  signal.id = signal_id;
  signal.name = signal_node.attribute("name").as_string("");
  signal.s = s;
  signal.t = t;
  signal.z_offset = signal_node.attribute("zOffset").as_double(0.0);
  signal.h_offset = signal_node.attribute("hOffset").as_double(0.0);
  signal.roll = signal_node.attribute("roll").as_double(0.0);
  signal.pitch = signal_node.attribute("pitch").as_double(0.0);

  const std::string_view orientation_str = signal_node.attribute("orientation").value();
  if (const auto orientation_opt = FromString<ast::Orientation>(orientation_str)) {
    signal.orientation = *orientation_opt;
  } else {
    throw InvalidAttributeError("<signal id=\"" + signal_id + "\"> has invalid orientation=\"" +
                                std::string(orientation_str) + "\"");
  }

  signal.dynamic = signal_node.attribute("dynamic").as_bool(false);
  signal.country = signal_node.attribute("country").as_string("");
  signal.country_revision = signal_node.attribute("countryRevision").as_string("");
  signal.type = signal_node.attribute("type").as_string();
  signal.subtype = signal_node.attribute("subtype").as_string();
  signal.value = signal_node.attribute("value").as_double(0.0);
  signal.unit = signal_node.attribute("unit").as_string("");
  signal.height = signal_node.attribute("height").as_double(0.0);
  signal.width = signal_node.attribute("width").as_double(0.0);
  signal.text = signal_node.attribute("text").as_string("");
  signal.temporary = signal_node.attribute("temporary").as_bool(false);
  signal.invalidated = signal_node.attribute("invalidated").as_bool(false);

  // Parse dependencies
  pugi::xml_node dep_node = signal_node.child("dependency");
  while (!dep_node.empty()) {
    signal.dependencies.push_back(ParseSignalDependency(dep_node, signal_id));
    dep_node = dep_node.next_sibling("dependency");
  }

  // Parse references
  pugi::xml_node ref_node = signal_node.child("reference");
  while (!ref_node.empty()) {
    signal.references.push_back(ParseSignalReferenceElement(ref_node, signal_id));
    ref_node = ref_node.next_sibling("reference");
  }

  // Parse validities
  signal.validities = ParseLaneValidities(signal_node);

  // Extensions
  static const std::unordered_set<std::string> kKnownSignalAttrs = {
      "id",   "name",      "s",           "t",       "zOffset", "hOffset",
      "roll", "pitch",     "orientation", "dynamic", "country", "countryRevision",
      "type", "subtype",   "value",       "unit",    "height",  "width",
      "text", "temporary", "invalidated"};
  signal.extensions = ParseExtensions(signal_node, kKnownSignalAttrs);

  return signal;
}

auto ParseSignalReference(pugi::xml_node sig_ref_node) -> ast::SignalReference {
  if (!sig_ref_node.attribute("id")) {
    throw MissingElementError("<signalReference> element is missing mandatory 'id' attribute");
  }
  std::string sig_ref_id = sig_ref_node.attribute("id").as_string();

  if (!sig_ref_node.attribute("s")) {
    throw MissingElementError("<signalReference id=\"" + sig_ref_id + "\"> is missing mandatory 's' attribute");
  }
  if (!sig_ref_node.attribute("t")) {
    throw MissingElementError("<signalReference id=\"" + sig_ref_id + "\"> is missing mandatory 't' attribute");
  }
  if (!sig_ref_node.attribute("orientation")) {
    throw MissingElementError("<signalReference id=\"" + sig_ref_id +
                              "\"> is missing mandatory 'orientation' attribute");
  }

  double s = sig_ref_node.attribute("s").as_double();
  if (s < 0.0) {
    throw InvalidAttributeError("<signalReference id=\"" + sig_ref_id + "\"> has invalid negative 's' attribute");
  }
  double t = sig_ref_node.attribute("t").as_double();

  ast::SignalReference sig_ref;
  sig_ref.id = sig_ref_id;
  sig_ref.s = s;
  sig_ref.t = t;
  sig_ref.z_offset = sig_ref_node.attribute("zOffset").as_double(0.0);

  const std::string_view orientation_str = sig_ref_node.attribute("orientation").value();
  if (const auto orientation_opt = FromString<ast::Orientation>(orientation_str)) {
    sig_ref.orientation = *orientation_opt;
  } else {
    throw InvalidAttributeError("<signalReference id=\"" + sig_ref_id + "\"> has invalid orientation=\"" +
                                std::string(orientation_str) + "\"");
  }

  // Parse validities
  sig_ref.validities = ParseLaneValidities(sig_ref_node);

  // Extensions
  static const std::unordered_set<std::string> kKnownSignalRefAttrs = {"id", "s", "t", "zOffset", "orientation"};
  sig_ref.extensions = ParseExtensions(sig_ref_node, kKnownSignalRefAttrs);

  return sig_ref;
}

auto ParseObjectCornerLocal(pugi::xml_node node) -> ast::ObjectCornerLocal {
  ast::ObjectCornerLocal corner;
  corner.u = node.attribute("u").as_double(0.0);
  corner.v = node.attribute("v").as_double(0.0);
  corner.z = node.attribute("z").as_double(0.0);
  corner.height = node.attribute("height").as_double(0.0);
  corner.id = node.attribute("id").as_uint(0);
  return corner;
}

auto ParseObjectCornerRoad(pugi::xml_node node) -> ast::ObjectCornerRoad {
  ast::ObjectCornerRoad corner;
  corner.s = node.attribute("s").as_double(0.0);
  corner.t = node.attribute("t").as_double(0.0);
  corner.dz = node.attribute("dz").as_double(0.0);
  corner.height = node.attribute("height").as_double(0.0);
  corner.id = node.attribute("id").as_uint(0);
  return corner;
}

auto ParseObjectCurveLocal(pugi::xml_node node) -> ast::ObjectCurveLocal {
  ast::ObjectCurveLocal curve;
  curve.id = node.attribute("id").as_uint(0);
  curve.u = node.attribute("u").as_double(0.0);
  curve.v = node.attribute("v").as_double(0.0);
  curve.z = node.attribute("z").as_double(0.0);
  curve.height = node.attribute("height").as_double(0.0);
  curve.length = node.attribute("length").as_double(0.0);
  curve.hdg = node.attribute("hdg").as_double(0.0);
  return curve;
}

auto ParseOutlineMarking(pugi::xml_node node) -> ast::OutlineMarking {
  ast::OutlineMarking marking;
  marking.side = node.attribute("side").as_string("");
  marking.weight = node.attribute("weight").as_string("");
  marking.width = node.attribute("width").as_double(0.0);
  marking.color = node.attribute("color").as_string("");
  marking.z_offset = node.attribute("zOffset").as_double(0.0);
  marking.space_length = node.attribute("spaceLength").as_double(0.0);
  marking.line_length = node.attribute("lineLength").as_double(0.0);
  marking.start_offset = node.attribute("startOffset").as_double(0.0);
  marking.stop_offset = node.attribute("stopOffset").as_double(0.0);

  pugi::xml_node ref_node = node.child("cornerReference");
  while (!ref_node.empty()) {
    ast::CornerReference ref;
    ref.id = ref_node.attribute("id").as_uint(0);
    marking.corner_references.push_back(ref);
    ref_node = ref_node.next_sibling("cornerReference");
  }
  return marking;
}

auto ParseObjectOutline(pugi::xml_node node) -> ast::ObjectOutline {
  ast::ObjectOutline outline;
  outline.id = node.attribute("id").as_uint(0);
  outline.fill_type = node.attribute("fillType").as_string("");
  outline.outer = node.attribute("outer").as_bool(false);
  outline.closed = node.attribute("closed").as_bool(false);
  outline.lane_type = node.attribute("laneType").as_string("");

  pugi::xml_node corner_local = node.child("cornerLocal");
  while (!corner_local.empty()) {
    outline.corners_local.push_back(ParseObjectCornerLocal(corner_local));
    corner_local = corner_local.next_sibling("cornerLocal");
  }

  pugi::xml_node corner_road = node.child("cornerRoad");
  while (!corner_road.empty()) {
    outline.corners_road.push_back(ParseObjectCornerRoad(corner_road));
    corner_road = corner_road.next_sibling("cornerRoad");
  }

  pugi::xml_node curve_local = node.child("curveLocal");
  while (!curve_local.empty()) {
    outline.curves_local.push_back(ParseObjectCurveLocal(curve_local));
    curve_local = curve_local.next_sibling("curveLocal");
  }

  pugi::xml_node markings_node = node.child("markings");
  if (!markings_node.empty()) {
    pugi::xml_node marking_node = markings_node.child("marking");
    while (!marking_node.empty()) {
      outline.markings.push_back(ParseOutlineMarking(marking_node));
      marking_node = marking_node.next_sibling("marking");
    }
  }

  return outline;
}

auto ParseObjectBorder(pugi::xml_node node) -> ast::ObjectBorder {
  ast::ObjectBorder border;
  border.width = node.attribute("width").as_double(0.0);
  border.type = node.attribute("type").as_string("");
  border.outline_id = node.attribute("outlineId").as_uint(0);
  border.use_complete_outline = node.attribute("useCompleteOutline").as_bool(false);

  pugi::xml_node ref_node = node.child("cornerReference");
  while (!ref_node.empty()) {
    ast::CornerReference ref;
    ref.id = ref_node.attribute("id").as_uint(0);
    border.corner_references.push_back(ref);
    ref_node = ref_node.next_sibling("cornerReference");
  }
  return border;
}

auto ParseObjectRepeat(pugi::xml_node node) -> ast::ObjectRepeat {
  ast::ObjectRepeat repeat;
  repeat.s = node.attribute("s").as_double(0.0);
  repeat.length = node.attribute("length").as_double(0.0);
  repeat.distance = node.attribute("distance").as_double(0.0);
  repeat.t_start = node.attribute("tStart").as_double(0.0);
  repeat.t_end = node.attribute("tEnd").as_double(0.0);
  repeat.height_start = node.attribute("heightStart").as_double(0.0);
  repeat.height_end = node.attribute("heightEnd").as_double(0.0);
  repeat.z_offset_start = node.attribute("zOffsetStart").as_double(0.0);
  repeat.z_offset_end = node.attribute("zOffsetEnd").as_double(0.0);
  repeat.width_start = node.attribute("widthStart").as_double(0.0);
  repeat.width_end = node.attribute("widthEnd").as_double(0.0);
  repeat.length_start = node.attribute("lengthStart").as_double(0.0);
  repeat.length_end = node.attribute("lengthEnd").as_double(0.0);
  repeat.radius_start = node.attribute("radiusStart").as_double(0.0);
  repeat.radius_end = node.attribute("radiusEnd").as_double(0.0);
  repeat.detach_from_reference_line = node.attribute("detachFromReferenceLine").as_bool(false);
  repeat.b_t = node.attribute("bT").as_double(0.0);
  repeat.c_t = node.attribute("cT").as_double(0.0);
  repeat.d_t = node.attribute("dT").as_double(0.0);
  return repeat;
}

auto ParseObject(pugi::xml_node node) -> ast::Object {
  if (!node.attribute("id")) {
    throw MissingElementError("<object> element is missing mandatory 'id' attribute");
  }
  std::string id = node.attribute("id").as_string();

  if (!node.attribute("s")) {
    throw MissingElementError("<object id=\"" + id + "\"> is missing mandatory 's' attribute");
  }
  if (!node.attribute("t")) {
    throw MissingElementError("<object id=\"" + id + "\"> is missing mandatory 't' attribute");
  }
  if (!node.attribute("orientation")) {
    throw MissingElementError("<object id=\"" + id + "\"> is missing mandatory 'orientation' attribute");
  }

  ast::Object obj;
  obj.id = id;
  obj.s = node.attribute("s").as_double();
  obj.t = node.attribute("t").as_double();

  const std::string_view orient_str = node.attribute("orientation").value();
  if (const auto orient_opt = FromString<ast::Orientation>(orient_str)) {
    obj.orientation = *orient_opt;
  } else {
    throw InvalidAttributeError("<object id=\"" + id + "\"> has invalid orientation=\"" + std::string(orient_str) +
                                "\"");
  }

  if (node.attribute("type") != nullptr) {
    const std::string_view type_str = node.attribute("type").value();
    if (const auto type_opt = FromString<ast::ObjectType>(type_str)) {
      obj.type = *type_opt;
    } else {
      throw InvalidAttributeError("<object id=\"" + id + "\"> has invalid type=\"" + std::string(type_str) + "\"");
    }
  } else {
    obj.type = ast::ObjectType::kNone;
  }

  obj.subtype = node.attribute("subtype").as_string("");
  obj.name = node.attribute("name").as_string("");
  obj.z_offset = node.attribute("zOffset").as_double(0.0);
  obj.roll = node.attribute("roll").as_double(0.0);
  obj.pitch = node.attribute("pitch").as_double(0.0);
  obj.hdg = node.attribute("hdg").as_double(0.0);
  obj.height = node.attribute("height").as_double(0.0);
  obj.length = node.attribute("length").as_double(0.0);
  obj.width = node.attribute("width").as_double(0.0);
  obj.dynamic = node.attribute("dynamic").as_bool(false);
  obj.valid_length = node.attribute("validLength").as_double(0.0);
  obj.perp_to_road = node.attribute("perpToRoad").as_bool(false);
  obj.radius = node.attribute("radius").as_double(0.0);
  obj.temporary = node.attribute("temporary").as_bool(false);
  obj.invalidated = node.attribute("invalidated").as_bool(false);

  // Repeats
  pugi::xml_node repeat_node = node.child("repeat");
  while (!repeat_node.empty()) {
    obj.repeats.push_back(ParseObjectRepeat(repeat_node));
    repeat_node = repeat_node.next_sibling("repeat");
  }

  // Outlines (can be nested under <outlines> or directly under <object>)
  pugi::xml_node outlines_node = node.child("outlines");
  if (!outlines_node.empty()) {
    pugi::xml_node outline_node = outlines_node.child("outline");
    while (!outline_node.empty()) {
      obj.outlines.push_back(ParseObjectOutline(outline_node));
      outline_node = outline_node.next_sibling("outline");
    }
  }
  pugi::xml_node outline_node = node.child("outline");
  while (!outline_node.empty()) {
    obj.outlines.push_back(ParseObjectOutline(outline_node));
    outline_node = outline_node.next_sibling("outline");
  }

  // Borders
  pugi::xml_node borders_node = node.child("borders");
  if (!borders_node.empty()) {
    pugi::xml_node border_node = borders_node.child("border");
    while (!border_node.empty()) {
      obj.borders.push_back(ParseObjectBorder(border_node));
      border_node = border_node.next_sibling("border");
    }
  }

  // Parking Space
  pugi::xml_node ps_node = node.child("parkingSpace");
  if (!ps_node.empty()) {
    ast::ParkingSpace parking_space;
    parking_space.access = ps_node.attribute("access").as_string("");
    parking_space.restrictions = ps_node.attribute("restrictions").as_string("");
    obj.parking_space = parking_space;
  }

  // Skeleton
  pugi::xml_node skeleton_node = node.child("skeleton");
  if (!skeleton_node.empty()) {
    ast::ObjectSkeleton skeleton;
    pugi::xml_node poly_node = skeleton_node.child("polyline");
    while (!poly_node.empty()) {
      ast::SkeletonPolyline polyline;
      polyline.id = poly_node.attribute("id").as_uint(0);

      pugi::xml_node vl_node = poly_node.child("vertexLocal");
      while (!vl_node.empty()) {
        ast::PolylineVertexLocal vertex_local;
        vertex_local.u = vl_node.attribute("u").as_double(0.0);
        vertex_local.v = vl_node.attribute("v").as_double(0.0);
        vertex_local.z = vl_node.attribute("z").as_double(0.0);
        vertex_local.radius = vl_node.attribute("radius").as_double(0.0);
        vertex_local.intersection_point = vl_node.attribute("intersectionPoint").as_bool(false);
        vertex_local.id = vl_node.attribute("id").as_uint(0);
        polyline.vertices_local.push_back(vertex_local);
        vl_node = vl_node.next_sibling("vertexLocal");
      }

      pugi::xml_node vr_node = poly_node.child("vertexRoad");
      while (!vr_node.empty()) {
        ast::PolylineVertexRoad vertex_road;
        vertex_road.s = vr_node.attribute("s").as_double(0.0);
        vertex_road.t = vr_node.attribute("t").as_double(0.0);
        vertex_road.dz = vr_node.attribute("dz").as_double(0.0);
        vertex_road.radius = vr_node.attribute("radius").as_double(0.0);
        vertex_road.intersection_point = vr_node.attribute("intersectionPoint").as_bool(false);
        vertex_road.id = vr_node.attribute("id").as_uint(0);
        polyline.vertices_road.push_back(vertex_road);
        vr_node = vr_node.next_sibling("vertexRoad");
      }

      skeleton.polylines.push_back(polyline);
      poly_node = poly_node.next_sibling("polyline");
    }
    obj.skeleton = skeleton;
  }

  // Validity
  obj.validities = ParseLaneValidities(node);

  // Surface
  pugi::xml_node surface_node = node.child("surface");
  if (!surface_node.empty()) {
    ast::ObjectSurface surface;
    pugi::xml_node crg_node = surface_node.child("CRG");
    if (!crg_node.empty()) {
      ast::ObjectSurfaceCrg crg;
      crg.file = crg_node.attribute("file").as_string("");
      crg.hide_road_surface_crg = crg_node.attribute("hideRoadSurfaceCRG").as_bool(false);
      crg.z_scale = crg_node.attribute("zScale").as_double(0.0);
      surface.crg = crg;
    }
    obj.surface = surface;
  }

  // Materials
  pugi::xml_node material_node = node.child("material");
  while (!material_node.empty()) {
    ast::ObjectMaterial material;
    material.surface = material_node.attribute("surface").as_string("");
    material.friction = material_node.attribute("friction").as_double(0.0);
    material.roughness = material_node.attribute("roughness").as_double(0.0);
    material.road_mark_color = material_node.attribute("roadMarkColor").as_string("");
    obj.materials.push_back(material);
    material_node = material_node.next_sibling("material");
  }

  static const std::unordered_set<std::string> kKnownObjectAttrs = {
      "id",      "type",        "subtype",    "name",        "s",         "t",          "zOffset",
      "roll",    "pitch",       "hdg",        "orientation", "height",    "length",     "width",
      "dynamic", "validLength", "perpToRoad", "radius",      "temporary", "invalidated"};
  obj.extensions = ParseExtensions(node, kKnownObjectAttrs);

  return obj;
}

auto ParseObjectReference(pugi::xml_node node) -> ast::ObjectReference {
  if (!node.attribute("id")) {
    throw MissingElementError("<objectReference> element is missing mandatory 'id' attribute");
  }
  std::string id = node.attribute("id").as_string();

  if (!node.attribute("s")) {
    throw MissingElementError("<objectReference id=\"" + id + "\"> is missing mandatory 's' attribute");
  }
  if (!node.attribute("t")) {
    throw MissingElementError("<objectReference id=\"" + id + "\"> is missing mandatory 't' attribute");
  }
  if (!node.attribute("orientation")) {
    throw MissingElementError("<objectReference id=\"" + id + "\"> is missing mandatory 'orientation' attribute");
  }

  ast::ObjectReference obj_ref;
  obj_ref.id = id;
  obj_ref.s = node.attribute("s").as_double();
  obj_ref.t = node.attribute("t").as_double();
  obj_ref.z_offset = node.attribute("zOffset").as_double(0.0);
  obj_ref.valid_length = node.attribute("validLength").as_double(0.0);

  const std::string_view orient_str = node.attribute("orientation").value();
  if (const auto orient_opt = FromString<ast::Orientation>(orient_str)) {
    obj_ref.orientation = *orient_opt;
  } else {
    throw InvalidAttributeError("<objectReference id=\"" + id + "\"> has invalid orientation=\"" +
                                std::string(orient_str) + "\"");
  }

  obj_ref.validities = ParseLaneValidities(node);

  static const std::unordered_set<std::string> kKnownObjectRefAttrs = {"id",      "s",           "t",
                                                                       "zOffset", "validLength", "orientation"};
  obj_ref.extensions = ParseExtensions(node, kKnownObjectRefAttrs);

  return obj_ref;
}

auto ParseDocument(const pugi::xml_document& doc) -> ast::AbstractSyntaxTree {
  const pugi::xml_node root = doc.child("OpenDRIVE");
  if (!root) {
    throw MissingElementError("Missing <OpenDRIVE> root element");
  }

  const pugi::xml_node header_node = root.child("header");
  if (!header_node) {
    throw MissingElementError("Missing <header> element");
  }

  ast::AbstractSyntaxTree ast_tree;
  ast_tree.header = ParseHeader(header_node);

  pugi::xml_node road_node = root.child("road");
  while (!road_node.empty()) {
    ast::Road road;
    road.id = road_node.attribute("id").as_string("");
    if (road.id.empty()) {
      throw MissingElementError("<road> element is missing mandatory 'id' attribute");
    }
    if (!road_node.attribute("length")) {
      throw MissingElementError("<road id=\"" + road.id + "\"> is missing mandatory 'length' attribute");
    }
    road.length = road_node.attribute("length").as_double(0.0);
    road.junction = road_node.attribute("junction").as_string("-1");
    if (const auto rule_attr = road_node.attribute("rule")) {
      const std::string_view rule_str = rule_attr.value();
      if (const auto rule_opt = FromString<ast::TrafficRule>(rule_str)) {
        road.rule = *rule_opt;
      } else {
        throw InvalidAttributeError("<road id=\"" + road.id + "\"> has invalid rule=\"" + std::string(rule_str) + "\"");
      }
    } else {
      road.rule = ast::TrafficRule::kRht;
    }
    if (road_node.attribute("name") != nullptr) {
      road.name = road_node.attribute("name").as_string();
    } else {
      road.name = std::nullopt;
    }

    // PlanView Geometries
    const pugi::xml_node plan_view_node = road_node.child("planView");
    if (plan_view_node.empty()) {
      throw MissingElementError("<road id=\"" + road.id + "\"> is missing mandatory <planView> element");
    }
    pugi::xml_node geom_node = plan_view_node.child("geometry");
    while (!geom_node.empty()) {
      road.plan_view.push_back(ParseGeometry(geom_node));
      geom_node = geom_node.next_sibling("geometry");
    }
    std::ranges::sort(road.plan_view, [](const ast::GeometryRecord& lhs, const ast::GeometryRecord& rhs) -> bool {
      return lhs.s < rhs.s;
    });

    // Elevation & Lateral Profiles
    road.elevation_profile = ParseElevationProfile(road_node.child("elevationProfile"));
    road.lateral_profile = ParseLateralProfile(road_node.child("lateralProfile"));

    // Lanes
    road.lanes = ParseLanes(road_node.child("lanes"));
    std::ranges::sort(road.lanes.sections,
                      [](const ast::LaneSection& lhs, const ast::LaneSection& rhs) -> bool { return lhs.s < rhs.s; });

    // Road Types
    pugi::xml_node type_node = road_node.child("type");
    while (!type_node.empty()) {
      ast::RoadTypeRecord record;
      record.s = type_node.attribute("s").as_double(0.0);
      if (const auto type_attr = type_node.attribute("type")) {
        const std::string_view type_str = type_attr.value();
        if (const auto type_opt = FromString<ast::RoadType>(type_str)) {
          record.type = *type_opt;
        } else {
          record.type = ast::RoadType::kUnknown;
        }
      } else {
        record.type = ast::RoadType::kUnknown;
      }
      road.types.push_back(record);
      type_node = type_node.next_sibling("type");
    }
    std::ranges::sort(road.types, [](const ast::RoadTypeRecord& lhs, const ast::RoadTypeRecord& rhs) -> bool {
      return lhs.s < rhs.s;
    });

    // Bridges
    pugi::xml_node bridge_node = road_node.child("bridge");
    while (!bridge_node.empty()) {
      road.bridges.push_back(ParseBridge(bridge_node));
      bridge_node = bridge_node.next_sibling("bridge");
    }
    std::ranges::sort(road.bridges,
                      [](const ast::Bridge& lhs, const ast::Bridge& rhs) -> bool { return lhs.s < rhs.s; });

    // Tunnels
    pugi::xml_node tunnel_node = road_node.child("tunnel");
    while (!tunnel_node.empty()) {
      road.tunnels.push_back(ParseTunnel(tunnel_node));
      tunnel_node = tunnel_node.next_sibling("tunnel");
    }
    std::ranges::sort(road.tunnels,
                      [](const ast::Tunnel& lhs, const ast::Tunnel& rhs) -> bool { return lhs.s < rhs.s; });

    // Objects & Object References
    pugi::xml_node objects_node = road_node.child("objects");
    if (!objects_node.empty()) {
      pugi::xml_node object_node = objects_node.child("object");
      while (!object_node.empty()) {
        road.objects.push_back(ParseObject(object_node));
        object_node = object_node.next_sibling("object");
      }
      std::ranges::sort(road.objects,
                        [](const ast::Object& lhs, const ast::Object& rhs) -> bool { return lhs.s < rhs.s; });

      pugi::xml_node object_ref_node = objects_node.child("objectReference");
      while (!object_ref_node.empty()) {
        road.object_references.push_back(ParseObjectReference(object_ref_node));
        object_ref_node = object_ref_node.next_sibling("objectReference");
      }
      std::ranges::sort(
          road.object_references,
          [](const ast::ObjectReference& lhs, const ast::ObjectReference& rhs) -> bool { return lhs.s < rhs.s; });
    }

    // Signals
    const pugi::xml_node signals_node = road_node.child("signals");
    if (!signals_node.empty()) {
      pugi::xml_node signal_node = signals_node.child("signal");
      while (!signal_node.empty()) {
        road.signals.push_back(ParseSignal(signal_node));
        signal_node = signal_node.next_sibling("signal");
      }
      std::ranges::sort(road.signals,
                        [](const ast::Signal& lhs, const ast::Signal& rhs) -> bool { return lhs.s < rhs.s; });

      pugi::xml_node sig_ref_node = signals_node.child("signalReference");
      while (!sig_ref_node.empty()) {
        road.signal_references.push_back(ParseSignalReference(sig_ref_node));
        sig_ref_node = sig_ref_node.next_sibling("signalReference");
      }
      std::ranges::sort(
          road.signal_references,
          [](const ast::SignalReference& lhs, const ast::SignalReference& rhs) -> bool { return lhs.s < rhs.s; });
    }

    // Extensions
    static const std::unordered_set<std::string> kNownRoadAttrs = {"id", "length", "junction", "rule", "name"};
    road.extensions = ParseExtensions(road_node, kNownRoadAttrs);

    ast_tree.roads.push_back(road);
    road_node = road_node.next_sibling("road");
  }

  pugi::xml_node junction_node = root.child("junction");
  while (!junction_node.empty()) {
    ast_tree.junctions.push_back(ParseJunction(junction_node));
    junction_node = junction_node.next_sibling("junction");
  }

  return ast_tree;
}

}  // namespace

auto ParseString(std::string_view xml_content) -> ast::AbstractSyntaxTree {
  pugi::xml_document doc;
  const pugi::xml_parse_result result = doc.load_buffer(xml_content.data(), xml_content.size());
  if (!result) {
    throw XmlParseError(std::string("Failed to parse XML from string: ") + result.description());
  }
  return ParseDocument(doc);
}

auto ParseFile(const std::filesystem::path& file_path) -> ast::AbstractSyntaxTree {
  pugi::xml_document doc;
  const pugi::xml_parse_result result = doc.load_file(file_path.c_str());
  if (!result) {
    throw XmlParseError(std::string("Failed to parse XML from file: ") + result.description());
  }
  return ParseDocument(doc);
}

}  // namespace strada::parser
