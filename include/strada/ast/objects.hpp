// SPDX-License-Identifier: BSL-1.0

#pragma once

#include <cstdint>
#include <optional>
#include <strada/ast/extensions.hpp>
#include <string>
#include <vector>

namespace strada::ast {

/// ASAM OpenDRIVE object types (e_objectType).
enum class ObjectType : std::uint8_t {
  kNone = 0,
  kObstacle,
  kCar,
  kPole,
  kTree,
  kVegetation,
  kBarrier,
  kBuilding,
  kParkingSpace,
  kPatch,
  kRailing,
  kTrafficIsland,
  kCrosswalk,
  kStreetLamp,
  kGantry,
  kSoundBarrier,
  kVan,
  kBus,
  kTrailer,
  kBike,
  kMotorbike,
  kTram,
  kTrain,
  kPedestrian,
  kWind,
  kRoadMark,
  kRoadSurface
};

/// Validity direction of an object or signal relative to reference line (e_orientation).
enum class Orientation : std::uint8_t {
  kNone = 0,  ///< "none" - Valid in both directions / no direction restriction
  kPlus,      ///< "+" - Valid in the direction of the reference line
  kMinus      ///< "-" - Valid in the opposite direction
};

/// ASAM OpenDRIVE layer types (e_layerType).
enum class LayerType : std::uint8_t { kPermanent = 0, kTemporary };

/// ASAM OpenDRIVE tunnel types (e_tunnelType).
enum class TunnelType : std::uint8_t { kStandard = 0, kUnderpass };

/// ASAM OpenDRIVE bridge types (e_bridgeType).
enum class BridgeType : std::uint8_t { kBrick = 0, kConcrete, kSteel, kWood };

/// Lane validity range for objects and signals (t_road_objects_object_laneValidity).
struct LaneValidity {
  int from_lane{};
  int to_lane{};
  LayerType layer{LayerType::kPermanent};
};

/// Local Cartesian corner definition (t_road_objects_object_outlines_outline_cornerLocal).
struct ObjectCornerLocal {
  double u{};
  double v{};
  double z{};
  double height{};
  std::uint32_t id{};
};

/// Road-relative corner definition (t_road_objects_object_outlines_outline_cornerRoad).
struct ObjectCornerRoad {
  double s{};
  double t{};
  double dz{};
  double height{};
  std::uint32_t id{};
};

/// Local curve definition for outline (t_road_objects_object_outlines_outline_curveLocal).
struct ObjectCurveLocal {
  std::uint32_t id{};
  double u{};
  double v{};
  double z{};
  double height{};
  double length{};
  double hdg{};
};

/// Reference to a corner in the same outline (t_road_objects_object_markings_marking_cornerReference).
struct CornerReference {
  std::uint32_t id{};
};

/// Marking definition for outlines (t_road_objects_object_markings_marking).
struct OutlineMarking {
  std::string side;
  std::string weight;
  double width{};
  std::string color;
  double z_offset{};
  double space_length{};
  double line_length{};
  double start_offset{};
  double stop_offset{};
  std::vector<CornerReference> corner_references;
};

/// Outline definition for objects (t_road_objects_object_outlines_outline).
struct ObjectOutline {
  std::uint32_t id{};
  std::string fill_type;
  bool outer{false};
  bool closed{false};
  std::string lane_type;
  std::vector<ObjectCornerLocal> corners_local;
  std::vector<ObjectCornerRoad> corners_road;
  std::vector<ObjectCurveLocal> curves_local;
  std::vector<OutlineMarking> markings;
};

/// Border definition for objects (t_road_objects_object_borders_border).
struct ObjectBorder {
  double width{};
  std::string type;
  std::uint32_t outline_id{};
  bool use_complete_outline{false};
  std::vector<CornerReference> corner_references;
};

/// Repeat rule definition for objects (t_road_objects_object_repeat).
struct ObjectRepeat {
  double s{};
  double length{};
  double distance{};
  double t_start{};
  double t_end{};
  double height_start{};
  double height_end{};
  double z_offset_start{};
  double z_offset_end{};
  double width_start{};
  double width_end{};
  double length_start{};
  double length_end{};
  double radius_start{};
  double radius_end{};
  bool detach_from_reference_line{false};
  double b_t{};
  double c_t{};
  double d_t{};
};

/// Access restrictions for parking space (t_road_objects_object_parkingSpace).
struct ParkingSpace {
  std::string access;
  std::string restrictions;
};

/// Vertex definitions for skeleton polyline (t_road_objects_object_skeleton_polyline_vertex*).
struct PolylineVertexLocal {
  double u{};
  double v{};
  double z{};
  double radius{};
  bool intersection_point{false};
  std::uint32_t id{};
};

/// Road-relative vertex definition for skeleton polyline (t_road_objects_object_skeleton_polyline_vertexRoad).
struct PolylineVertexRoad {
  double s{};
  double t{};
  double dz{};
  double radius{};
  bool intersection_point{false};
  std::uint32_t id{};
};

/// Polyline definition for skeleton (t_road_objects_object_skeleton_polyline).
struct SkeletonPolyline {
  std::uint32_t id{};
  std::vector<PolylineVertexLocal> vertices_local;
  std::vector<PolylineVertexRoad> vertices_road;
};

/// Skeleton definition (t_road_objects_object_skeleton).
struct ObjectSkeleton {
  std::vector<SkeletonPolyline> polylines;
};

/// Surface CRG definition (t_road_objects_object_surface_CRG).
struct ObjectSurfaceCrg {
  std::string file;
  bool hide_road_surface_crg{false};
  double z_scale{};
};

/// Surface definition (t_road_objects_object_surface).
struct ObjectSurface {
  std::optional<ObjectSurfaceCrg> crg;
};

/// Material definition (t_road_objects_object_material).
struct ObjectMaterial {
  std::string surface;
  double friction{};
  double roughness{};
  std::string road_mark_color;
};

/// Primary Object element (t_road_objects_object).
struct Object {
  std::string id;
  ObjectType type{ObjectType::kNone};
  std::string subtype;
  std::string name;
  double s{};
  double t{};
  double z_offset{};
  double roll{};
  double pitch{};
  double hdg{};
  Orientation orientation{Orientation::kNone};
  double height{};
  double length{};
  double width{};
  bool dynamic{false};
  double valid_length{};
  bool perp_to_road{false};
  double radius{};
  bool temporary{false};
  bool invalidated{false};

  // Nested Child elements
  std::vector<ObjectRepeat> repeats;
  std::vector<ObjectOutline> outlines;
  std::vector<ObjectBorder> borders;
  std::optional<ParkingSpace> parking_space;
  std::optional<ObjectSkeleton> skeleton;
  std::vector<LaneValidity> validities;
  std::optional<ObjectSurface> surface;
  std::vector<ObjectMaterial> materials;

  Extensions extensions;
};

/// Object reference definition (t_road_objects_objectReference).
struct ObjectReference {
  std::string id;
  double s{};
  double t{};
  double z_offset{};
  double valid_length{};
  Orientation orientation{Orientation::kNone};
  std::vector<LaneValidity> validities;

  Extensions extensions;
};

/// Bridge definition (t_road_objects_bridge).
struct Bridge {
  std::string id;
  double s{};
  double length{};
  std::optional<std::string> name;
  BridgeType type;
  std::vector<LaneValidity> validities;

  Extensions extensions;
};

/// Tunnel definition (t_road_objects_tunnel).
struct Tunnel {
  std::string id;
  double s{};
  double length{};
  std::optional<std::string> name;
  TunnelType type;
  std::optional<double> lighting;
  std::optional<double> daylight;
  std::vector<LaneValidity> validities;

  Extensions extensions;
};

}  // namespace strada::ast
