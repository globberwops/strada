#include <algorithm>
#include <limits>
#include <map>
#include <pugixml.hpp>
#include <sstream>
#include <strada/ast/objects.hpp>
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

  const pugi::xml_node kGeoRefNode = header_node.child("geoReference");
  if (!kGeoRefNode.empty()) {
    header.geo_reference = kGeoRefNode.child_value();
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

  if (const pugi::xml_node kLineNode = geom_node.child("line"); !kLineNode.empty()) {
    geom.shape = ast::Line{};
  } else if (const pugi::xml_node kSpiralNode = geom_node.child("spiral"); !kSpiralNode.empty()) {
    ast::Spiral spiral;
    spiral.curv_start = kSpiralNode.attribute("curvStart").as_double(0.0);
    spiral.curv_end = kSpiralNode.attribute("curvEnd").as_double(0.0);
    geom.shape = spiral;
  } else if (const pugi::xml_node kArcNode = geom_node.child("arc"); !kArcNode.empty()) {
    ast::Arc arc;
    arc.curvature = kArcNode.attribute("curvature").as_double(0.0);
    geom.shape = arc;
  } else if (const pugi::xml_node kPoly3Node = geom_node.child("poly3"); !kPoly3Node.empty()) {
    ast::Poly3 poly3;
    poly3.a = kPoly3Node.attribute("a").as_double(0.0);
    poly3.b = kPoly3Node.attribute("b").as_double(0.0);
    poly3.c = kPoly3Node.attribute("c").as_double(0.0);
    poly3.d = kPoly3Node.attribute("d").as_double(0.0);
    geom.shape = poly3;
  } else if (const pugi::xml_node kParamPoly3Node = geom_node.child("paramPoly3"); !kParamPoly3Node.empty()) {
    ast::ParamPoly3 param_poly3;
    param_poly3.a_u = kParamPoly3Node.attribute("aU").as_double(0.0);
    param_poly3.b_u = kParamPoly3Node.attribute("bU").as_double(0.0);
    param_poly3.c_u = kParamPoly3Node.attribute("cU").as_double(0.0);
    param_poly3.d_u = kParamPoly3Node.attribute("dU").as_double(0.0);
    param_poly3.a_v = kParamPoly3Node.attribute("aV").as_double(0.0);
    param_poly3.b_v = kParamPoly3Node.attribute("bV").as_double(0.0);
    param_poly3.c_v = kParamPoly3Node.attribute("cV").as_double(0.0);
    param_poly3.d_v = kParamPoly3Node.attribute("dV").as_double(0.0);

    const std::string kPRangeStr = kParamPoly3Node.attribute("pRange").as_string("normalized");
    if (kPRangeStr == "arcLength") {
      param_poly3.p_range = ast::PRange::kArcLength;
    } else {
      param_poly3.p_range = ast::PRange::kNormalized;
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

  const pugi::xml_node kCssNode = lat_prof_node.child("crossSectionSurface");
  if (!kCssNode.empty()) {
    ast::CrossSectionSurface css;
    const pugi::xml_node kTOffsetNode = kCssNode.child("tOffset");
    if (!kTOffsetNode.empty()) {
      pugi::xml_node coeff_node = kTOffsetNode.child("coefficients");
      while (!coeff_node.empty()) {
        css.t_offset.push_back(ParseCoefficient(coeff_node));
        coeff_node = coeff_node.next_sibling("coefficients");
      }
    }
    const pugi::xml_node kSurfaceStripsNode = kCssNode.child("surfaceStrips");
    if (!kSurfaceStripsNode.empty()) {
      pugi::xml_node strip_node = kSurfaceStripsNode.child("strip");
      while (!strip_node.empty()) {
        ast::CrossSectionSurfaceStrip strip;
        strip.id = strip_node.attribute("id").as_int(0);
        strip.mode = strip_node.attribute("mode").as_string("independent");

        const pugi::xml_node kWidthNode = strip_node.child("width");
        if (!kWidthNode.empty()) {
          pugi::xml_node coeff_node = kWidthNode.child("coefficients");
          while (!coeff_node.empty()) {
            strip.width.push_back(ParseCoefficient(coeff_node));
            coeff_node = coeff_node.next_sibling("coefficients");
          }
        }
        const pugi::xml_node kConstNode = strip_node.child("constant");
        if (!kConstNode.empty()) {
          pugi::xml_node coeff_node = kConstNode.child("coefficients");
          while (!coeff_node.empty()) {
            strip.constant.push_back(ParseCoefficient(coeff_node));
            coeff_node = coeff_node.next_sibling("coefficients");
          }
        }
        const pugi::xml_node kLinearNode = strip_node.child("linear");
        if (!kLinearNode.empty()) {
          pugi::xml_node coeff_node = kLinearNode.child("coefficients");
          while (!coeff_node.empty()) {
            strip.linear.push_back(ParseCoefficient(coeff_node));
            coeff_node = coeff_node.next_sibling("coefficients");
          }
        }
        const pugi::xml_node kQuadNode = strip_node.child("quadratic");
        if (!kQuadNode.empty()) {
          pugi::xml_node coeff_node = kQuadNode.child("coefficients");
          while (!coeff_node.empty()) {
            strip.quadratic.push_back(ParseCoefficient(coeff_node));
            coeff_node = coeff_node.next_sibling("coefficients");
          }
        }
        const pugi::xml_node kCubicNode = strip_node.child("cubic");
        if (!kCubicNode.empty()) {
          pugi::xml_node coeff_node = kCubicNode.child("coefficients");
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
  lane.type = lane_node.attribute("type").as_string("");
  lane.level = lane_node.attribute("level").as_bool(false);

  const pugi::xml_node kLinkNode = lane_node.child("link");
  if (!kLinkNode.empty()) {
    const pugi::xml_node kPredNode = kLinkNode.child("predecessor");
    if (!kPredNode.empty()) {
      lane.predecessor = kPredNode.attribute("id").as_int(0);
    }
    const pugi::xml_node kSuccNode = kLinkNode.child("successor");
    if (!kSuccNode.empty()) {
      lane.successor = kSuccNode.attribute("id").as_int(0);
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
    const std::string kTypeStr = seg_node.attribute("type").as_string("");
    if (kTypeStr == "joint") {
      segment.type = ast::JunctionSegmentType::kJoint;
    } else {
      segment.type = ast::JunctionSegmentType::kLane;
    }

    segment.road_id = seg_node.attribute("roadId").as_string("");

    if (segment.type == ast::JunctionSegmentType::kLane) {
      if (seg_node.attribute("boundaryLane") != nullptr) {
        segment.boundary_lane = seg_node.attribute("boundaryLane").as_int();
      }
      segment.s_start = ParseBoundaryCoordinate(seg_node.attribute("sStart").as_string("begin"));
      segment.s_end = ParseBoundaryCoordinate(seg_node.attribute("sEnd").as_string("end"));
    } else {
      const std::string kCpStr = seg_node.attribute("contactPoint").as_string("start");
      segment.contact_point = (kCpStr == "end") ? ast::ContactPoint::kEnd : ast::ContactPoint::kStart;
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
  junction.name = junction_node.attribute("name").as_string("");
  junction.type = junction_node.attribute("type").as_string("");

  pugi::xml_node conn_node = junction_node.child("connection");
  while (!conn_node.empty()) {
    ast::Connection conn;
    conn.id = conn_node.attribute("id").as_string("");
    conn.incoming_road = conn_node.attribute("incomingRoad").as_string("");
    conn.connecting_road = conn_node.attribute("connectingRoad").as_string("");
    const std::string kCpStr = conn_node.attribute("contactPoint").as_string("start");
    conn.contact_point = (kCpStr == "end") ? ast::ContactPoint::kEnd : ast::ContactPoint::kStart;

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

  const pugi::xml_node kBoundaryNode = junction_node.child("boundary");
  if (!kBoundaryNode.empty()) {
    junction.boundary = ParseBoundary(kBoundaryNode);
  }

  static const std::unordered_set<std::string> kNownJunctionAttrs = {"id", "name", "type"};
  junction.extensions = ParseExtensions(junction_node, kNownJunctionAttrs);
  return junction;
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

  double s = bridge_node.attribute("s").as_double();
  if (s < 0.0) {
    throw InvalidAttributeError("<bridge id=\"" + bridge_id + "\"> has invalid negative 's' attribute");
  }
  double length = bridge_node.attribute("length").as_double();
  if (length < 0.0) {
    throw InvalidAttributeError("<bridge id=\"" + bridge_id + "\"> has invalid negative 'length' attribute");
  }

  ast::Bridge bridge;
  bridge.id = bridge_id;
  bridge.s = s;
  bridge.length = length;
  bridge.name = bridge_node.attribute("name").as_string("");
  bridge.type = bridge_node.attribute("type").as_string("");

  pugi::xml_node valid_node = bridge_node.child("valid");
  while (!valid_node.empty()) {
    ast::LaneValidity validity;
    validity.from_lane = valid_node.attribute("fromLane").as_int(0);
    validity.to_lane = valid_node.attribute("toLane").as_int(0);
    validity.layer = valid_node.attribute("layer").as_string("");
    bridge.validities.push_back(validity);
    valid_node = valid_node.next_sibling("valid");
  }

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

  double s = tunnel_node.attribute("s").as_double();
  if (s < 0.0) {
    throw InvalidAttributeError("<tunnel id=\"" + tunnel_id + "\"> has invalid negative 's' attribute");
  }
  double length = tunnel_node.attribute("length").as_double();
  if (length < 0.0) {
    throw InvalidAttributeError("<tunnel id=\"" + tunnel_id + "\"> has invalid negative 'length' attribute");
  }

  ast::Tunnel tunnel;
  tunnel.id = tunnel_id;
  tunnel.s = s;
  tunnel.length = length;
  tunnel.name = tunnel_node.attribute("name").as_string("");
  tunnel.type = tunnel_node.attribute("type").as_string("");
  tunnel.lighting = tunnel_node.attribute("lighting").as_double(0.0);
  tunnel.daylight = tunnel_node.attribute("daylight").as_double(0.0);

  pugi::xml_node valid_node = tunnel_node.child("valid");
  while (!valid_node.empty()) {
    ast::LaneValidity validity;
    validity.from_lane = valid_node.attribute("fromLane").as_int(0);
    validity.to_lane = valid_node.attribute("toLane").as_int(0);
    validity.layer = valid_node.attribute("layer").as_string("");
    tunnel.validities.push_back(validity);
    valid_node = valid_node.next_sibling("valid");
  }

  static const std::unordered_set<std::string> kKnownTunnelAttrs = {"id",   "s",        "length",  "name",
                                                                    "type", "lighting", "daylight"};
  tunnel.extensions = ParseExtensions(tunnel_node, kKnownTunnelAttrs);

  return tunnel;
}

auto ParseDocument(const pugi::xml_document& doc) -> ast::AbstractSyntaxTree {
  const pugi::xml_node kRoot = doc.child("OpenDRIVE");
  if (!kRoot) {
    throw MissingElementError("Missing <OpenDRIVE> root element");
  }

  const pugi::xml_node kHeaderNode = kRoot.child("header");
  if (!kHeaderNode) {
    throw MissingElementError("Missing <header> element");
  }

  ast::AbstractSyntaxTree ast_tree;
  ast_tree.header = ParseHeader(kHeaderNode);

  pugi::xml_node road_node = kRoot.child("road");
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
    const std::string kRuleStr = road_node.attribute("rule").as_string("RHT");
    if (kRuleStr == "LHT") {
      road.rule = ast::TrafficRule::kLht;
    } else {
      road.rule = ast::TrafficRule::kRht;
    }
    road.name = road_node.attribute("name").as_string("");

    // PlanView Geometries
    const pugi::xml_node kPlanViewNode = road_node.child("planView");
    if (kPlanViewNode.empty()) {
      throw MissingElementError("<road id=\"" + road.id + "\"> is missing mandatory <planView> element");
    }
    pugi::xml_node geom_node = kPlanViewNode.child("geometry");
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

    // Extensions
    static const std::unordered_set<std::string> kNownRoadAttrs = {"id", "length", "junction", "rule", "name"};
    road.extensions = ParseExtensions(road_node, kNownRoadAttrs);

    ast_tree.roads.push_back(road);
    road_node = road_node.next_sibling("road");
  }

  pugi::xml_node junction_node = kRoot.child("junction");
  while (!junction_node.empty()) {
    ast_tree.junctions.push_back(ParseJunction(junction_node));
    junction_node = junction_node.next_sibling("junction");
  }

  return ast_tree;
}

}  // namespace

auto ParseString(std::string_view xml_content) -> ast::AbstractSyntaxTree {
  pugi::xml_document doc;
  const pugi::xml_parse_result kResult = doc.load_buffer(xml_content.data(), xml_content.size());
  if (!kResult) {
    throw XmlParseError(std::string("Failed to parse XML from string: ") + kResult.description());
  }
  return ParseDocument(doc);
}

auto ParseFile(const std::filesystem::path& file_path) -> ast::AbstractSyntaxTree {
  pugi::xml_document doc;
  const pugi::xml_parse_result kResult = doc.load_file(file_path.c_str());
  if (!kResult) {
    throw XmlParseError(std::string("Failed to parse XML from file: ") + kResult.description());
  }
  return ParseDocument(doc);
}

}  // namespace strada::parser
