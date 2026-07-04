// SPDX-License-Identifier: BSL-1.0

#pragma once

#include <strada/ast/abstract_syntax_tree.hpp>
#include <strada/cpm/ids.hpp>
#include <string>
#include <vector>

namespace strada::cpm {
class CompiledPhysicsModel;
struct QueryContext;
}  // namespace strada::cpm

namespace strada::tess {

/// Represents a 3D vertex position using 32-bit floats for GPU rendering efficiency.
struct Vertex {
  float x{};  ///< Cartesian X coordinate (meters).
  float y{};  ///< Cartesian Y coordinate (meters).
  float z{};  ///< Cartesian Z coordinate (meters).
};

/// Represents a static index and vertex buffer mesh of a lane or road surface.
struct Mesh {
  std::vector<Vertex> vertices;        ///< Flat list of 3D vertices.
  std::vector<std::uint32_t> indices;  ///< Indices defining triangle layout.
  cpm::RoadId road_id{};               ///< Compiled ID of the parent road.
  cpm::LaneId lane_id{};               ///< Compiled ID of the lane.
  std::string lane_type;               ///< The structural type of the lane (e.g., "driving", "sidewalk").
};

/// Represents a static set of points forming a line boundary or marking.
struct Polyline {
  std::vector<Vertex> vertices;   ///< Consecutive 3D points forming the line.
  cpm::RoadId road_id{};          ///< Compiled ID of the parent road.
  int original_lane_id{0};        ///< The original index ID from the XODR source (e.g. -1, 1, 0).
  bool is_reference_line{false};  ///< True if this polyline represents the road's center reference line.
  std::string marking_type;       ///< Styling description of the marking (e.g., "solid", "broken", "none").
};

/// Represents the compiled 3D triangulation and outline of a junction boundary.
struct JunctionBoundaryGeometry {
  std::vector<Vertex> vertices;          ///< Triangulated surface vertices of the boundary area.
  std::vector<std::uint32_t> indices;    ///< Triangulation indices.
  std::vector<Vertex> outline_vertices;  ///< Winding outline vertices (closed loop).
  std::string junction_id;               ///< ID of the parent junction.
};

/// Holds the output layers of tessellated road network geometries.
class Tessellator {
 public:
  /// Constructs the Tessellator layer from the Logical AST.
  ///
  /// Internally instantiates a temporary CompiledPhysicsModel to evaluate geometries
  /// with the specified chord error tolerance.
  ///
  /// \param map The logical C++ abstract syntax tree of the map.
  /// \param chord_error The maximum chord error tolerance in meters (e.g., 0.5m).
  Tessellator(const ast::AbstractSyntaxTree& map, double chord_error);

  /// Returns the flat list of generated road surface meshes.
  [[nodiscard]] auto Meshes() const noexcept -> const std::vector<Mesh>& { return meshes_; }

  /// Returns the flat list of generated boundary polylines.
  [[nodiscard]] auto Polylines() const noexcept -> const std::vector<Polyline>& { return polylines_; }

  /// Returns the flat list of generated junction boundary geometries.
  [[nodiscard]] auto JunctionBoundaries() const noexcept -> const std::vector<JunctionBoundaryGeometry>& {
    return junction_boundaries_;
  }

 private:
  [[nodiscard]] static auto ComputeSamplingStations(const ast::Road& road, double chord_error) -> std::vector<double>;

  void TessellateReferenceLine(cpm::RoadId road_id, const std::vector<double>& stations,
                               const cpm::CompiledPhysicsModel& model, cpm::QueryContext& ctx);

  void TessellateLaneSections(const ast::Road& road, cpm::RoadId road_id, const std::vector<double>& stations,
                              const cpm::CompiledPhysicsModel& model, cpm::QueryContext& ctx,
                              const ast::AbstractSyntaxTree& map);

  void TessellateJunctionBoundaries(const ast::AbstractSyntaxTree& map, const cpm::CompiledPhysicsModel& model,
                                    cpm::QueryContext& ctx, double chord_error);

  std::vector<Mesh> meshes_;
  std::vector<Polyline> polylines_;
  std::vector<JunctionBoundaryGeometry> junction_boundaries_;
};

}  // namespace strada::tess
